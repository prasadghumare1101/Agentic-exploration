#!/usr/bin/env python3
"""Lightweight MJPEG streamer for the FPV panel.

Source is a ROS 2 camera topic from Gazebo (there is no webcam on a SITL rig).
Serves multipart/x-mixed-replace on :8080/video_feed so the browser can bind it
straight to an <img src> - frames never enter React state, which is what keeps
the dashboard's memory flat.

Zero-transcode path: image_transport already publishes JPEG on
<topic>/compressed, so those bytes are forwarded to the browser untouched. No
decode, no re-encode, near-zero CPU. The raw rgb8 topic is only used as a
fallback when the compressed topic is unavailable.

Config (env):
  AERO_CAMERA_TOPIC   base topic (default /front_camera/image_raw)
  AERO_FPS            max frames/sec pushed to the browser (default 15)
  AERO_JPEG_QUALITY   quality for the raw-topic fallback only (default 50)

Must run with ROS 2 sourced. Run:  python3 video_streamer.py
"""
from __future__ import annotations

import os
import threading
import time

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, Response
from fastapi.responses import StreamingResponse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String as StringMsg

BASE_TOPIC = os.environ.get("AERO_CAMERA_TOPIC", "/front_camera/image_raw")
TARGET_FPS = float(os.environ.get("AERO_FPS", 15))
JPEG_QUALITY = int(os.environ.get("AERO_JPEG_QUALITY", 50))
STALE_AFTER_S = 2.0
WIDTH, HEIGHT = 640, 360


class LatestFrame:
    """Single-slot buffer. Old frames are dropped, never queued, so a slow
    browser can never make this grow."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._stamp = 0.0

    def set(self, jpeg: bytes) -> None:
        with self._lock:
            self._jpeg = jpeg
            self._stamp = time.monotonic()

    def get(self) -> bytes | None:
        with self._lock:
            if self._jpeg is None or time.monotonic() - self._stamp > STALE_AFTER_S:
                return None
            return self._jpeg


FRAME = LatestFrame()


def _placeholder(msg: str = "WAITING FOR CAMERA") -> bytes:
    img = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
    img[:] = (28, 28, 32)
    cv2.putText(img, msg, (28, HEIGHT // 2 - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (120, 120, 130), 2)
    cv2.putText(img, BASE_TOPIC, (28, HEIGHT // 2 + 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (90, 90, 100), 1)
    ok, buf = cv2.imencode(".jpg", img)
    return buf.tobytes() if ok else b""


# ---------------------------------------------------------------------------- #
# Per-drone feeds.
#
# Each vehicle exposes TWO views:
#   raw       — /iris_N/front_camera/image_raw  (the camera itself)
#   annotated — /px4_N/operator/image           (detector output: boxes + track ids)
# The console asks for view=auto and we serve the ANNOTATED feed while that drone is
# actually tracking (follow state DETECT/LOCK/FOLLOW), RAW otherwise — so detections are
# on screen exactly when they exist, with no operator fiddling.
# ---------------------------------------------------------------------------- #

N_DRONES = max(1, int(os.environ.get("AEGIS_DRONES", "1")))
TRACKING_STATES = {"DETECT", "LOCK", "LOCKED", "FOLLOW"}

FRAMES: dict = {}          # FRAMES[drone][view] -> LatestFrame
FOLLOW: dict = {}          # FOLLOW[drone] -> follow-state string

# Live viewers, keyed by (drone, view). Frames are only JPEG-encoded for streams somebody
# is actually watching: with 3 drones the node would otherwise encode 6 video streams
# continuously (raw + annotated each) while the console displays exactly one. That was pure
# CPU burn on a machine already running 3x PX4, Gazebo and 3 detectors.
VIEWERS: dict = {}
VIEWER_TTL = 5.0           # a stream stays "watched" briefly after the client goes away


def _watching(drone: str, view: str) -> bool:
    t = VIEWERS.get((drone, view), 0.0)
    return (time.monotonic() - t) < VIEWER_TTL


def _mark_watching(drone: str, view: str) -> None:
    VIEWERS[(drone, view)] = time.monotonic()


def _slot(drone: str, view: str) -> LatestFrame:
    return FRAMES.setdefault(drone, {}).setdefault(view, LatestFrame())


class CameraNode(Node):
    """Subscribes to raw + annotated feeds for every vehicle (compressed preferred)."""

    def __init__(self) -> None:
        super().__init__("gcs_video_streamer")
        # Gazebo image_transport publishers are best-effort; a RELIABLE reader
        # would silently never match them.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._got_compressed: dict = {}

        for i in range(1, N_DRONES + 1):
            drone = f"px4_{i}"
            FOLLOW[drone] = "IDLE"
            # Same convention in both modes: camera i is /iris_i, detector output is /px4_i.
            # No single-vehicle special case, so a swarm feed can never be confused with a
            # single-drone one.
            sources = {
                "raw": f"/iris_{i}/front_camera/image_raw",
                "annotated": f"/{drone}/operator/image",
            }

            for view, topic in sources.items():
                self.create_subscription(
                    CompressedImage, f"{topic}/compressed",
                    (lambda m, d=drone, v=view: self._on_compressed(d, v, m)), qos)
                self.create_subscription(
                    Image, topic,
                    (lambda m, d=drone, v=view: self._on_raw(d, v, m)), qos)

            self.create_subscription(
                StringMsg, f"/{drone}/follow_state",
                (lambda m, d=drone: FOLLOW.__setitem__(d, (m.data or "").strip().upper())),
                10)
            self.get_logger().info(
                f"{drone}: raw={sources['raw']}  annotated={sources['annotated']}")

    def _on_compressed(self, drone: str, view: str, msg: CompressedImage) -> None:
        # format is e.g. "rgb8; jpeg compressed bgr8" -> already JPEG, forward as-is
        if "jpeg" in msg.format.lower():
            self._got_compressed[(drone, view)] = True
            _slot(drone, view).set(bytes(msg.data))

    def _on_raw(self, drone: str, view: str, msg: Image) -> None:
        # only used when the compressed topic is absent
        if self._got_compressed.get((drone, view)):
            return
        # Skip the decode+encode entirely if nobody is watching this stream.
        if not (_watching(drone, view) or _watching(drone, "auto")):
            return
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1)
            if msg.encoding == "rgb8":
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            ok, buf = cv2.imencode(
                ".jpg", arr, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            if ok:
                _slot(drone, view).set(buf.tobytes())
        except (ValueError, cv2.error):
            pass


app = FastAPI(title="AEROMAST Video Streamer")


def _pick(drone: str, view: str) -> bytes | None:
    """Resolve view=auto against the drone's follow state, with graceful fallback."""
    want_annotated = view == "annotated" or (view == "auto" and FOLLOW.get(drone, "IDLE") in TRACKING_STATES)
    order = ("annotated", "raw") if want_annotated else ("raw", "annotated")
    for v in order:
        jpeg = _slot(drone, v).get()
        if jpeg:
            return jpeg
    return None


def generate_frames(drone: str, view: str):
    period = 1.0 / TARGET_FPS
    blank = _placeholder()
    while True:
        start = time.monotonic()
        # keep this stream (and the concrete view auto resolves to) marked as watched
        _mark_watching(drone, view)
        if view == "auto":
            _mark_watching(drone, "annotated" if FOLLOW.get(drone, "IDLE") in TRACKING_STATES else "raw")
        jpeg = _pick(drone, view) or blank
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
        time.sleep(max(0.0, period - (time.monotonic() - start)))


@app.get("/video_feed")
def video_feed(drone: str = "px4_1", view: str = "auto") -> StreamingResponse:
    """MJPEG for one vehicle. view: auto (mission-driven) | raw | annotated."""
    return StreamingResponse(
        generate_frames(drone, view),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/snapshot")
def snapshot(drone: str = "px4_1", view: str = "auto") -> Response:
    """Single current frame. Used by the CAPTURE mission command - one cheap
    GET beats holding a second image subscription open in the bridge."""
    jpeg = _pick(drone, view)
    if jpeg is None:
        return Response(content=_placeholder("NO FRAME"), media_type="image/jpeg",
                        status_code=503)
    return Response(content=jpeg, media_type="image/jpeg")


@app.get("/health")
def health() -> dict:
    return {
        "ok": True,
        "drones": N_DRONES,
        "follow": FOLLOW,
        "receiving": {
            d: {v: (_slot(d, v).get() is not None) for v in ("raw", "annotated")}
            for d in FRAMES
        },
    }


def _spin_ros() -> None:
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    threading.Thread(target=_spin_ros, daemon=True).start()
    print(f"[INFO] Video streamer on :8080  (source {BASE_TOPIC})")
    uvicorn.run(app, host="0.0.0.0", port=8080, log_level="warning")
