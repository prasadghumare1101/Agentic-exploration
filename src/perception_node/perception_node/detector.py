#!/usr/bin/env python3
"""Vision target detector + tracker — one per drone, NON-BLOCKING.

Runs a VisDrone-trained YOLOv8n on the drone's DOWNWARD camera and publishes tracked
targets of the user-configured class(es). Design points that fix the old detector:

  * inference runs in a WORKER THREAD, not the ROS callback — the image callback only
    stashes the latest frame, so the executor never stalls on YOLO.
  * ultralytics `.track()` gives PERSISTENT track IDs across frames (ByteTrack), so a
    downstream follower can lock ONE target instead of re-picking the biggest box.
  * each detection is localised to a world/BEV position (top-down pixel projection using
    the camera intrinsics + the drone's altitude & pose; refined by depth when available).
  * target class(es) are configurable (`target_classes` param) and re-configurable live on
    `/{drone}/set_target` — "look for a person" vs "a car" is config, not code.
  * on a hit it also publishes a throttled ANNOTATED image + a text report for the human
    operator, and never spams raw images otherwise.

Outputs (per drone):
  /{drone}/detections        vision_msgs/Detection2DArray  (bbox + score + track id; id in
                                                            results[0].hypothesis.class_id
                                                            as "<class>#<id>", and bbox)
  /{drone}/target_world       geometry_msgs/PointStamped    (BEV world pos of best target)
  /{drone}/operator/image     sensor_msgs/Image             (annotated, throttled)
  /{drone}/operator/report    std_msgs/String               (human-readable target report)
"""

import math
import re
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from vision_msgs.msg import (Detection2DArray, Detection2D,
                             ObjectHypothesisWithPose, BoundingBox2D)
from cv_bridge import CvBridge
from px4_msgs.msg import VehicleLocalPosition

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=5)

# Sentinel class the supervisor parks un-tasked drones on (control_api.IDLE_CLASS).
# Matches nothing in any model, so it means "detect nothing" - as opposed to an EMPTY
# target set, which by long-standing convention here means "detect everything".
IDLE_CLASS = '__idle__'


class Detector(Node):

    def __init__(self):
        super().__init__('detector')
        self.drone_id = self.declare_parameter('drone_id', 'px4_1').value
        k = self._inst()
        # target class(es): a comma list or ROS string-array; matched against model names.
        tc = self.declare_parameter('target_classes', 'pedestrian,people').value
        self.targets = self._parse_targets(tc)
        self.conf = float(self.declare_parameter('confidence', 0.35).value)
        # YOLO inference image size. 640 is the size the COCO weights were trained at, so it is
        # both the cheapest and the best-matched choice: inference cost scales with the pixel
        # count, and 960 cost 2.25x as much per frame on a GPU shared with gazebo's renderer.
        self.imgsz = int(self.declare_parameter('imgsz', 640).value)
        # MIN box size (fraction of image height) — reject tiny sliver detections that are almost
        # always texture noise, not a real trackable target. 0 disables.
        self.min_box_h = float(self.declare_parameter('min_box_h', 0.06).value)
        # ByteTrack config: default 'bytetrack.yaml' (new_track_thresh 0.6) needs strong
        # detections to START a track. For a weakly-detected target (e.g. a small synthetic
        # actor) pass a low-threshold config so real-but-low-confidence detections still get a
        # persistent id (otherwise every frame is untracked #-1 and no lock is possible).
        self.tracker_cfg = str(self.declare_parameter('tracker', 'bytetrack.yaml').value)
        self.rate = float(self.declare_parameter('infer_rate', 5.0).value)   # Hz worker runs
        self.report_period = float(self.declare_parameter('report_period', 1.0).value)
        self.model_path = str(self.declare_parameter(
            'model', '/home/prasad/ws_swarm/models/visdrone_yolov8n.pt').value)
        cam = self.declare_parameter(
            'camera_topic', f'/iris_{k}/downward_depth_camera/image_raw').value
        info_topic = self.declare_parameter(
            'camera_info_topic', f'/iris_{k}/downward_depth_camera/camera_info').value

        from ultralytics import YOLO
        self.model = YOLO(self.model_path)
        self.names = self.model.names
        # FP16 on the GPU. Halves weight/activation VRAM and roughly doubles throughput on
        # consumer cards, with no measurable effect on detection at these confidences. This
        # matters here because the GPU is shared with gazebo's renderer — VRAM freed by the
        # detectors is VRAM the camera sensors get back.
        try:
            import torch
            self.half = bool(torch.cuda.is_available())
        except Exception:
            self.half = False
        self.bridge = CvBridge()

        self._frame = None          # latest BGR frame (shared with worker)
        self._frame_stamp = None
        self._lock = threading.Lock()
        # Camera intrinsics for the BEV projection. Default to values computed for the
        # downward camera (640x480, ~60deg HFOV -> fx~554); if the camera publishes a real
        # CameraInfo it overrides these. Falling back to params means BEV/world-follow works
        # even when camera_info is absent (the sim's downward depth cam doesn't publish it).
        self.fx = float(self.declare_parameter('fx', 554.0).value)
        self.fy = float(self.declare_parameter('fy', 554.0).value)
        self.cx = float(self.declare_parameter('cx', 320.0).value)
        self.cy = float(self.declare_parameter('cy', 240.0).value)
        # BEV/world projection is an OPTIONAL operator/logging side-output only. The follower
        # runs image-space servoing and never consumes it, so it is OFF by default — no _bev()
        # call, no /target_world publish, no world point packed into detections.
        self.publish_world = bool(self.declare_parameter('publish_world', False).value)
        self.pose = None            # (north, east, up, yaw) world
        self._last_report = 0.0

        # ---- CONTINUOUS visual tracker (DJI-ActiveTrack style) ----
        # YOLO is intermittent on weak/synthetic targets (id drops every few frames). A CSRT
        # visual tracker, seeded ONCE from a YOLO detection, then follows those pixels EVERY
        # frame -> a stable, continuous lock that does not drop between YOLO hits. YOLO only
        # re-seeds periodically (or when CSRT fails / drifts off the detection), keeping the
        # class identity honest while the tracker provides frame-to-frame continuity.
        self.use_csrt = bool(self.declare_parameter('continuous_tracker', True).value)
        self.reseed_period = float(self.declare_parameter('reseed_period', 1.5).value)  # s
        # identity-linking across brief losses: keep the SAME id if YOLO re-detects within
        # link_gate px of the last box within link_grace seconds (kills id churn on gaps)
        self.link_gate = float(self.declare_parameter('link_gate', 160.0).value)   # px
        self.link_grace = float(self.declare_parameter('link_grace', 2.0).value)   # s
        self._trk = None            # active cv2 tracker (or None)
        self._trk_id = None         # stable id we assign to the continuously-tracked target
        self._trk_name = None       # class name of the tracked target
        self._trk_score = 0.0       # last YOLO confidence when (re)seeded
        self._last_seed = 0.0       # time of last YOLO seed
        self._last_bbox = None      # last good bbox (for cross-gap identity linking)
        self._last_bbox_t = 0.0
        self._next_id = 1000        # stable-id counter for CSRT locks (distinct from ByteTrack)

        self.create_subscription(Image, cam, self._on_image, SENSOR_QOS)
        self.create_subscription(CameraInfo, info_topic, self._on_info, SENSOR_QOS)
        self.create_subscription(VehicleLocalPosition,
                                 f'/px4_{k}/fmu/out/vehicle_local_position_v1',
                                 self._on_pose, PX4_QOS)
        self.create_subscription(String, f'/{self.drone_id}/set_target',
                                 self._on_set_target, 10)

        self.det_pub = self.create_publisher(Detection2DArray, f'/{self.drone_id}/detections', 10)
        self.world_pub = self.create_publisher(PointStamped, f'/{self.drone_id}/target_world', 10)
        self.img_pub = self.create_publisher(Image, f'/{self.drone_id}/operator/image', 1)
        self.report_pub = self.create_publisher(String, f'/{self.drone_id}/operator/report', 10)

        # NON-BLOCKING: YOLO runs on its own thread, never on the ROS executor.
        self._run = True
        self._worker = threading.Thread(target=self._infer_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f'detector up for {self.drone_id}: targets={sorted(self.targets)} '
            f'model={self.model_path.split("/")[-1]} cam={cam}')

    # ---------- helpers ----------
    def _inst(self):
        m = re.search(r'(\d+)$', str(self.drone_id))
        return int(m.group(1)) if m else 1

    def _parse_targets(self, tc):
        if isinstance(tc, (list, tuple)):
            items = list(tc)
        else:
            items = str(tc).split(',')
        return {s.strip().lower() for s in items if str(s).strip()}

    # ---------- inputs (cheap, non-blocking) ----------
    def _on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        with self._lock:
            self._frame = frame
            self._frame_stamp = msg.header.stamp

    def _on_info(self, m):
        if m.k[0] > 0:      # a real CameraInfo overrides the fallback intrinsics
            self.fx, self.fy, self.cx, self.cy = m.k[0], m.k[4], m.k[2], m.k[5]

    def _on_pose(self, m):
        self.pose = (m.x, m.y, -m.z, m.heading)     # world north, east, up, yaw

    def _on_set_target(self, msg):
        self.targets = self._parse_targets(msg.data)
        self.get_logger().info(f'[{self.drone_id}] target reconfigured -> {sorted(self.targets)}')

    # ---------- BEV projection: image pixel -> world (E,N), target assumed on ground ----------
    def _bev(self, u, v):
        """Top-down projection: a downward camera at altitude H maps pixel offset from the
        principal point to a ground offset (approx; assumes flat ground under the drone)."""
        if self.pose is None or self.fx is None:
            return None
        pn, pe, up, yaw = self.pose
        if up < 1.0:
            return None
        # camera ray offset on the ground (metres), camera x=right(East-ish), y=down(South-ish)
        gx = (u - self.cx) / self.fx * up          # right of the drone, in metres
        gy = (v - self.cy) / self.fy * up          # forward/back in image, in metres
        # rotate the image-plane offset by the drone yaw (NED heading, CW from North) into world
        # image +y (down) points to the drone's BACK for a forward-mounted down-cam; use yaw
        east = pe + gx * math.cos(yaw) + gy * math.sin(yaw)
        north = pn - gx * math.sin(yaw) + gy * math.cos(yaw)
        return north, east

    # ---------- worker: YOLO + tracking off the ROS thread ----------
    def _idle(self):
        """True while this drone has been given no real target class.

        The supervisor parks un-tasked drones on the `__idle__` sentinel so they never
        auto-detect. Until now that was enforced by filtering YOLO's *output*, which meant an
        idle drone still ran a full inference pass every cycle — 3 drones x 5 Hz of GPU burnt
        producing results that were immediately thrown away. Gating here instead makes an
        un-tasked drone cost nothing, and the GPU goes to gazebo's cameras until a mission
        actually arms the vehicle.
        """
        return self.targets == {IDLE_CLASS}

    def _infer_loop(self):
        period = 1.0 / max(self.rate, 0.5)
        while self._run and rclpy.ok():
            t0 = time.time()
            if self._idle():
                time.sleep(period)
                continue
            with self._lock:
                frame = None if self._frame is None else self._frame.copy()
                stamp = self._frame_stamp
            if frame is None:
                time.sleep(period)
                continue
            try:
                if self.use_csrt:
                    self._csrt_step(frame, stamp)      # continuous CSRT lock (DJI-style)
                else:
                    res = self.model.track(frame, persist=True, conf=self.conf,
                                           imgsz=self.imgsz, tracker=self.tracker_cfg,
                                           half=self.half, verbose=False)[0]
                    self._publish(res, frame, stamp)
            except Exception as e:
                self.get_logger().warn(f'inference failed: {e}', throttle_duration_sec=5.0)
                time.sleep(period)
                continue
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)

    # ---------- CONTINUOUS visual tracking (CSRT seeded by YOLO) ----------
    @staticmethod
    def _new_tracker():
        if hasattr(cv2, 'TrackerCSRT_create'):
            return cv2.TrackerCSRT_create()
        return cv2.legacy.TrackerCSRT_create()

    @staticmethod
    def _iou(a, b):
        ax1, ay1, aw, ah = a; bx1, by1, bw, bh = b
        ax2, ay2, bx2, by2 = ax1 + aw, ay1 + ah, bx1 + bw, by1 + bh
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
        inter = iw * ih
        union = aw * ah + bw * bh - inter
        return inter / union if union > 0 else 0.0

    def _yolo_best_box(self, frame):
        """One YOLO detect pass -> best (x,y,w,h), score, name for a target class, or None.
        Rejects sub-min-height boxes (texture-noise false positives)."""
        res = self.model(frame, imgsz=self.imgsz, conf=self.conf,
                         half=self.half, verbose=False)[0]
        min_h = self.min_box_h * frame.shape[0]
        best = None
        for b in (res.boxes or []):
            name = self.names[int(b.cls)].lower()
            if self.targets and name not in self.targets:
                continue
            score = float(b.conf)
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
            if (y2 - y1) < min_h:           # too small -> almost certainly noise, skip
                continue
            if best is None or score > best[1]:
                best = ((x1, y1, x2 - x1, y2 - y1), score, name)
        return best

    def _csrt_step(self, frame, stamp):
        now = time.time()
        bbox = None                     # (x,y,w,h) of the tracked target this frame
        # 1) CONTINUOUS: advance the existing tracker on this frame (every frame -> no gaps)
        if self._trk is not None:
            ok, box = self._trk.update(frame)
            if ok:
                bbox = tuple(float(v) for v in box)
            else:
                self._trk = None        # tracker lost the target -> will try to re-seed below

        # 2) RE-SEED from YOLO: periodically (keep identity honest) or when we have no tracker
        need_seed = (self._trk is None) or (now - self._last_seed >= self.reseed_period)
        if need_seed:
            det = self._yolo_best_box(frame)
            if det is not None:
                ybox, score, name = det
                # CONTINUITY: keep the SAME id if the re-detection overlaps the live tracker box
                # OR (when CSRT had briefly failed) is close to the LAST known box within a grace
                # window. Only start a NEW id when there's genuinely no recent target to link to.
                ref = bbox if bbox is not None else (
                    self._last_bbox if (now - self._last_bbox_t) < self.link_grace else None)
                cont = False
                if ref is not None:
                    rcx, rcy = ref[0] + ref[2] / 2.0, ref[1] + ref[3] / 2.0
                    ycx, ycy = ybox[0] + ybox[2] / 2.0, ybox[1] + ybox[3] / 2.0
                    near = math.hypot(ycx - rcx, ycy - rcy) < self.link_gate
                    cont = self._iou(ybox, ref) > 0.1 or near
                if self._trk_id is None or not cont:
                    self._trk_id = self._next_id; self._next_id += 1
                    self.get_logger().info(
                        f'[{self.drone_id}] CSRT LOCK #{self._trk_id} {name} conf={score:.2f}')
                # (re)initialise CSRT on the crisp YOLO box -> corrects drift, keeps continuity
                self._trk = self._new_tracker()
                self._trk.init(frame, tuple(int(v) for v in ybox))
                bbox = ybox
                self._trk_name = name
                self._trk_score = score
                self._last_seed = now

        # remember the last good box for cross-gap identity linking
        if bbox is not None:
            self._last_bbox = bbox
            self._last_bbox_t = now
        elif self._trk_id is not None and (now - self._last_bbox_t) >= self.link_grace:
            self._trk_id = None         # target truly gone past the grace window -> allow new id

        # 3) PUBLISH the continuous bbox (stable id) + operator image every frame
        self._publish_box(bbox, frame, stamp)

    def _publish_box(self, bbox, frame, stamp):
        vis = frame.copy()
        out = Detection2DArray()
        if stamp is not None:
            out.header.stamp = stamp
        out.header.frame_id = f'{self.drone_id}/base_link'

        if bbox is not None and self._trk_id is not None:
            x, y, w, h = bbox
            ucx, vcy = x + w / 2.0, y + h / 2.0
            world = self._bev(ucx, vcy) if self.publish_world else None
            d = Detection2D()
            d.header = out.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = f'{self._trk_name}#{self._trk_id}'
            hyp.hypothesis.score = float(self._trk_score)
            if world is not None:
                hyp.pose.pose.position.x = float(world[1])
                hyp.pose.pose.position.y = float(world[0])
            d.results.append(hyp)
            bb = BoundingBox2D()
            bb.center.position.x = float(ucx); bb.center.position.y = float(vcy)
            bb.size_x = float(w); bb.size_y = float(h)
            d.bbox = bb
            out.detections.append(d)
            # draw the continuous lock for the operator window
            cv2.rectangle(vis, (int(x), int(y)), (int(x + w), int(y + h)), (0, 255, 0), 2)
            cv2.putText(vis, f'{self._trk_name}#{self._trk_id} {self._trk_score:.2f}',
                        (int(x), max(0, int(y) - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0), 2)
            now = time.time()
            if now - self._last_report >= self.report_period:
                self._last_report = now
                wtxt = f'world(N={world[0]:.1f},E={world[1]:.1f})' if world else 'world=?'
                self.report_pub.publish(String(
                    data=f'[{self.drone_id}] TARGET {self._trk_name}#{self._trk_id} '
                         f'conf={self._trk_score:.2f} {wtxt} (continuous)'))
                self.get_logger().info(
                    f'[{self.drone_id}] tracking {self._trk_name}#{self._trk_id} '
                    f'conf={self._trk_score:.2f} (CSRT)')
            if out.detections:
                self.det_pub.publish(out)
                if world is not None:
                    ps = PointStamped(); ps.header = out.header; ps.header.frame_id = 'map'
                    ps.point.x = float(world[1]); ps.point.y = float(world[0]); ps.point.z = 0.0
                    self.world_pub.publish(ps)

        # operator image every frame (continuous view, box drawn when locked)
        try:
            im = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            if stamp is not None:
                im.header.stamp = stamp
            im.header.frame_id = f'{self.drone_id}/base_link'
            self.img_pub.publish(im)
        except Exception:
            pass

    def _publish(self, res, frame, stamp):
        # LIVE annotated feed for the operator window — every inference, so the window shows
        # the camera view continuously with boxes/ids drawn whenever targets are present.
        try:
            vis = res.plot()
            im = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            if stamp is not None:
                im.header.stamp = stamp
            im.header.frame_id = f'{self.drone_id}/base_link'
            self.img_pub.publish(im)
        except Exception:
            pass

        out = Detection2DArray()
        if stamp is not None:
            out.header.stamp = stamp
        out.header.frame_id = f'{self.drone_id}/base_link'
        best = None          # (score, cx, cy, world, id, name)
        annotated = False
        for b in (res.boxes or []):
            name = self.names[int(b.cls)].lower()
            if self.targets and name not in self.targets:
                continue
            score = float(b.conf)
            tid = int(b.id) if b.id is not None else -1
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
            ucx, vcy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            world = self._bev(ucx, vcy) if self.publish_world else None

            d = Detection2D()
            d.header = out.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = f'{name}#{tid}'      # class + persistent track id
            hyp.hypothesis.score = score
            if world is not None:
                hyp.pose.pose.position.x = float(world[1])  # East
                hyp.pose.pose.position.y = float(world[0])  # North
            d.results.append(hyp)
            bb = BoundingBox2D()
            bb.center.position.x = ucx
            bb.center.position.y = vcy
            bb.size_x = x2 - x1
            bb.size_y = y2 - y1
            d.bbox = bb
            out.detections.append(d)
            if best is None or score > best[0]:
                best = (score, ucx, vcy, world, tid, name)

        if not out.detections:
            return
        self.det_pub.publish(out)

        # best target -> world/BEV point for nav/follow reasoning
        if best and best[3] is not None:
            ps = PointStamped()
            ps.header = out.header
            ps.header.frame_id = 'map'
            ps.point.x = float(best[3][1]); ps.point.y = float(best[3][0]); ps.point.z = 0.0
            self.world_pub.publish(ps)

        # operator report (throttled): annotated image + text
        now = time.time()
        if now - self._last_report >= self.report_period:
            self._last_report = now
            s, _, _, world, tid, name = best
            wtxt = f'world(N={world[0]:.1f},E={world[1]:.1f})' if world else 'world=?'
            self.report_pub.publish(String(
                data=f'[{self.drone_id}] TARGET {name}#{tid} conf={s:.2f} {wtxt} '
                     f'({len(out.detections)} in view)'))
            self.get_logger().info(f'[{self.drone_id}] tracking {name}#{tid} conf={s:.2f}')

    def destroy_node(self):
        self._run = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
