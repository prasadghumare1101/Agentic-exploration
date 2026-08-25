#!/usr/bin/env python3
"""AEGIS control API - the service that replaces the five terminals.

Owns two things and nothing else:
  1. process lifecycle for the simulation stack (uXRCE-DDS agent, PX4 SITL,
     rosbridge, telemetry node, video streamer),
  2. mission dispatch - takes a natural-language prompt from the dashboard and
     mission dispatch via the ws_swarm LLM planner (LLM -> validator -> MissionPlan).

It deliberately does NOT import rclpy or the pipeline packages: keeping this a
pure supervisor means a crash in a child process can never take the API down.

Run:  python3 control_api.py          (listens on :8000)
"""
from __future__ import annotations

import glob
import json
import math
import os
import tempfile
import re
import shutil
import signal
import subprocess
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# --------------------------------------------------------------------------- #
# Paths / configuration
# --------------------------------------------------------------------------- #

# AEGIS owns this backend. Nothing here reads from or writes to any other project.
SERVICES_DIR = Path(__file__).resolve().parent           # ~/aegis_gcs/backend
PIPELINE_ROOT = SERVICES_DIR.parent                      # ~/aegis_gcs (service cwd)
PX4_DIR = Path(os.environ.get("PX4_DIR", Path.home() / "PX4-Autopilot"))
ROS_SETUP = os.environ.get("ROS_SETUP", "/opt/ros/humble/setup.bash")
# The ROS workspace that provides the vehicles, perception and the LLM planner.
WS_SETUP = Path(os.environ.get("WS_SWARM", Path.home() / "ws_swarm")) / "install" / "setup.bash"

# PX4 gazebo-classic default home (Zurich). Override with PX4_HOME_LAT/LON.
HOME_LAT = float(os.environ.get("PX4_HOME_LAT", 47.397742))
HOME_LON = float(os.environ.get("PX4_HOME_LON", 8.545594))

LOG_LINES = 400
LOG_DIR = Path(__file__).resolve().parent / "logs"

WORLDS_DIR = (PX4_DIR / "Tools" / "simulation" / "gazebo-classic"
              / "sitl_gazebo-classic" / "worlds")
MODELS_DIR = (PX4_DIR / "Tools" / "simulation" / "gazebo-classic"
              / "sitl_gazebo-classic" / "models")

# Selected airframe + world + mission shape. "empty" is the fast default.
#   mode   : "single" (one vehicle, PX4 make target) | "swarm" (N vehicles via ws_swarm)
#   drones : vehicle count for swarm mode
#   moving : moving object spawned as the tracking target ("none" | person | car | ...)
SIM_CONFIG: Dict[str, object] = {
    "model": os.environ.get("PX4_MODEL", "iris"),
    "world": os.environ.get("PX4_WORLD", "empty"),
    "mode": os.environ.get("AEGIS_MODE", "single"),
    "drones": int(os.environ.get("AEGIS_DRONES", "1")),
    "moving": os.environ.get("AEGIS_MOVING", "none"),
    # Decentralised swarm: each drone runs a drone_agent that self-assigns formation slots
    # via a distributed auction (SlotBid), plus mission_supervisor for LLM replanning on
    # disruption. OFF = the coordinator assigns every slot centrally.
    "agentic": os.environ.get("AEGIS_AGENTIC", "0") in ("1", "true", "True"),
    # GPS-denied operation: navigate on SLAM alone. This also decides whether the per-drone
    # rtabmap (3D-LiDAR + RGB-D) stack is launched — without it the SLAM panel has no real
    # map to draw and falls back to odometry.
    "gps_denied": os.environ.get("AEGIS_GPS_DENIED", "0") in ("1", "true", "True"),
}

# Sentinel target class used while the stack is IDLE.
#
# ws_swarm's detector filters with `if self.targets and name not in self.targets: skip`, so an
# EMPTY target set means "no filter" — i.e. detect EVERYTHING. That is exactly why powering on
# used to make a drone chase the first person in frame. A class no model can ever emit gives
# the opposite, correct behaviour: the detector runs but publishes nothing until a mission
# arms it with a real class.
IDLE_CLASS = "__idle__"

# ---- ws_swarm integration (swarm missions + the LLM planner live there) ----
WS_SWARM = Path(os.environ.get("WS_SWARM", Path.home() / "ws_swarm"))
WS_SWARM_SETUP = WS_SWARM / "install" / "setup.bash"
DDS_PROFILE = WS_SWARM / "config" / "udp_only.xml"
SWARM_LAUNCHER = (PX4_DIR / "Tools" / "simulation" / "gazebo-classic"
                  / "sitl_multiple_run.sh")
DETECT_MODEL = WS_SWARM / "models" / "yolov8n.pt"

# Worlds that already contain a moving target actor, keyed by the object they carry.
# Selecting a moving object switches to the world that provides it, so "spawn a moving
# target" is one deterministic choice instead of runtime model injection.
MOVING_WORLDS: Dict[str, str] = {
    "person": "clear_follow",
    "car": "clear_follow",
    "truck": "clear_follow",
    # NOTE: no "person_only" entry. It used to map to a `ksql_airport_person` world that
    # does not exist in this repository, so _moving_objects() filtered it out and the
    # option silently never appeared in the console. Add the world first, then the entry.
}

# Gazebo hangs for minutes (or forever) trying to pull meshes from its online
# model database when a heavy world - baylands, ksql_airport, sonoma_raceway -
# references a model that is not cached locally. That fetch is the usual cause
# of a "stuck" launch. Everything PX4 needs already ships in the repo, so the
# database is disabled and the local paths are pinned instead.
SITL_ENV: Dict[str, str] = {
    "GAZEBO_MODEL_DATABASE_URI": "",
    "GAZEBO_MODEL_PATH": f"{MODELS_DIR}:{Path.home() / '.gazebo' / 'models'}",
    # CRITICAL: run px4 as `px4 -d` (no interactive pxh shell). Without this the
    # pxh prompt redraws itself forever when stdin is not a TTY, writing ANSI
    # escape codes at hundreds of MB/min. That burns CPU and fills the disk,
    # which is what actually starves Gazebo into "not responding".
    "NO_PXH": "1",
    # Drop the gzclient follow-camera plugin: less GUI load, fewer stalls.
    "PX4_NO_FOLLOW_MODE": "1",
}


def _dds_env() -> Dict[str, str]:
    """FastDDS transport, applied to EVERY service.

    All participants must agree on the transport or they simply never discover each other:
    the swarm nodes would be on UDP-only while telemetry/video/agent stayed on the default
    SHM+UDP, and the console would show a running sim with no data. UDP-only is also what
    keeps 3-vehicle SITL from exhausting shared-memory segments.
    """
    return {"FASTRTPS_DEFAULT_PROFILES_FILE": str(DDS_PROFILE)} if DDS_PROFILE.exists() else {}

# Hard ceiling on any captured service log. Defence in depth for the runaway
# case above - a log can never be allowed to consume the disk.
MAX_LOG_BYTES = 20 * 1024 * 1024

# Heavy worlds need far longer than `empty` before the DDS link appears.
SITL_READY_TIMEOUT_S = float(os.environ.get("PX4_READY_TIMEOUT", 180))


def _sitl_target() -> str:
    """PX4 make target. `model__world` (double underscore) selects a world."""
    model = SIM_CONFIG["model"]
    world = SIM_CONFIG["world"]
    if not world or world in ("default", "empty"):
        return f"gazebo-classic_{model}"
    return f"gazebo-classic_{model}__{world}"


def _list_worlds() -> List[str]:
    if not WORLDS_DIR.is_dir():
        return ["empty"]
    return sorted(p.stem for p in WORLDS_DIR.glob("*.world"))


def _list_models() -> List[str]:
    if not MODELS_DIR.is_dir():
        return ["iris"]
    return sorted(p.name for p in MODELS_DIR.iterdir()
                  if p.is_dir() and (p / "model.config").is_file())


def _sourced(cmd: str, with_ws: bool = False) -> List[str]:
    """Wrap a command so it runs inside a properly sourced ROS 2 shell."""
    parts = [f"source {ROS_SETUP}"]
    if with_ws and WS_SETUP.exists():
        parts.append(f"source {WS_SETUP}")
    parts.append(f"exec {cmd}")
    return ["bash", "-c", " && ".join(parts)]


# --------------------------------------------------------------------------- #
# Managed service definitions
# --------------------------------------------------------------------------- #

class Service:
    def __init__(self, name: str, argv: List[str], cwd: Path, detail: str,
                 env: Optional[Dict[str, str]] = None):
        self.name = name
        self.argv = argv
        self.cwd = cwd
        self.detail = detail
        self.env = env
        self.proc: Optional[subprocess.Popen] = None
        self.log_path: Optional[Path] = None
        self._log_fh = None

    @property
    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def start(self) -> None:
        if self.running:
            return
        merged = None
        if self.env:
            merged = {**os.environ, **self.env}
        # Capture output to a file. Discarding it (DEVNULL) makes a failed or
        # hung launch completely undiagnosable from the dashboard.
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        self.log_path = LOG_DIR / f"{self.name}.log"
        self._log_fh = self.log_path.open("w", buffering=1, encoding="utf-8",
                                          errors="replace")
        self.proc = subprocess.Popen(
            self.argv,
            cwd=str(self.cwd),
            env=merged,
            stdout=self._log_fh,
            stderr=subprocess.STDOUT,
            start_new_session=True,      # own process group -> killable as a tree
        )

    def tail(self, lines: int = 40) -> str:
        if not self.log_path or not self.log_path.is_file():
            return "(no log yet)"
        try:
            return "".join(self.log_path.read_text(
                encoding="utf-8", errors="replace").splitlines(keepends=True)[-lines:])
        except OSError as e:
            return f"(log unreadable: {e})"

    def stop(self) -> None:
        if not self.running or self.proc is None:
            self.proc = None
            return
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            try:
                self.proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                self.proc.wait(timeout=5)
        except (ProcessLookupError, PermissionError):
            pass
        finally:
            self.proc = None
            if self._log_fh:
                try:
                    self._log_fh.close()
                except OSError:
                    pass
                self._log_fh = None


SERVICES: Dict[str, Service] = {
    "agent": Service(
        "agent", ["MicroXRCEAgent", "udp4", "-p", "8888"], PIPELINE_ROOT,
        "udp4 :8888"),
    # argv/detail are rebuilt from SIM_CONFIG on every start (see _apply_sim_config)
    "sitl": Service(
        "sitl", ["bash", "-c", "exec make px4_sitl gazebo-classic_iris"], PX4_DIR,
        "iris / empty", env=SITL_ENV),
    "rosbridge": Service(
        "rosbridge",
        _sourced("ros2 launch rosbridge_server rosbridge_websocket_launch.xml"),
        PIPELINE_ROOT, "websocket :9090"),
    "telemetry": Service(
        "telemetry",
        _sourced(f"python3 {SERVICES_DIR / 'telemetry_node.py'}", with_ws=True),
        PIPELINE_ROOT, "/gcs/consolidated_telemetry"),
    "video": Service(
        "video",
        # needs rclpy: the FPV feed comes from a Gazebo ROS camera topic
        _sourced(f"python3 {SERVICES_DIR / 'video_streamer.py'}"),
        SERVICES_DIR, "MJPEG :8080"),
    # ws_swarm perception + follow stack. argv is rebuilt by _apply_sim_config; only
    # started for swarm missions (see START_ORDER below).
    "swarm": Service(
        "swarm", ["bash", "-c", "true"], PIPELINE_ROOT, "swarm perception+follow"),
}

# Order matters: agent before SITL so the DDS link comes up cleanly.
BASE_START_ORDER = ["agent", "sitl", "rosbridge", "telemetry", "video"]


def _start_order() -> List[str]:
    """The ws_swarm stack runs LAST in BOTH modes, so it binds to vehicles that already
    exist.

    It is required even for a single drone: it provides the mission executor
    (mission_supervisor + drone_controller) that turns a validated MissionPlan into flight,
    plus the detector/follower. Without it a plan is published to nobody — which is exactly
    the "stack offline - plan not dispatched" case.
    """
    return BASE_START_ORDER + ["swarm"]


# Backwards-compatible alias for existing references.
START_ORDER = BASE_START_ORDER


# --------------------------------------------------------------------------- #
# Pre-flight cleanup
#
# A stale gzserver still holding the old world is the usual cause of the
# "simulation is not responding / force quit" dialog, and of the laggy second
# run. Every start therefore reaps orphans and clears ONLY Gazebo scratch.
#
# NEVER TOUCHED:
#   ~/.gazebo/models            - contains the hand-modified `iris` model
#   PX4 .../sitl_gazebo-classic/models
#   PX4 build output
#   MicroXRCEAgent on a SERIAL port - that is the real drone hardware link
#
# Log purging destroys history, so it is a separate endpoint the operator must
# confirm explicitly. It never runs as part of starting the simulation.
# --------------------------------------------------------------------------- #

PROTECTED_PATHS = (
    Path.home() / ".gazebo" / "models",
    PX4_DIR / "Tools" / "simulation" / "gazebo-classic" / "sitl_gazebo-classic" / "models",
    PX4_DIR / "build",
)

# Reaped on start. "MicroXRCEAgent udp4" is matched in full ON PURPOSE: the bare
# binary name would also match `MicroXRCEAgent serial --dev /dev/ttyUSB0`, which
# is the physical drone link and must survive.
ORPHAN_PATTERNS = (
    "gzserver", "gzclient", "px4", "MicroXRCEAgent udp4",
    # ws_swarm stack nodes. These MUST be reaped too: they are launched into their own
    # process group, so stopping the launch parent leaves them alive. A surviving
    # coordinator_node from a previous run keeps publishing the last phase of its old plan
    # (e.g. the "return to home" rtl of a finished mission) onto /px4_N/task, and duplicate
    # drone_controllers then fight over the same vehicle — which looks exactly like
    # "the drone refuses to take off".
    "coordinator_node", "drone_controller", "safety_node", "heartbeat_node",
    "prompt_gateway", "mission_supervisor", "drone_agent",
    "perception_node/detector", "target_follow", "local_planner", "occupancy_mapper",
    "stage1_swarm.launch.py",
)

SCRATCH_GLOBS = ("/tmp/gazebo_*", f"/tmp/hsperfdata_{os.environ.get('USER', 'nobody')}")


def _is_protected(path: Path) -> bool:
    """True if the path is, contains, or lives inside anything protected."""
    try:
        rp = path.resolve()
    except OSError:
        return True                       # unresolvable -> refuse to touch it
    for guard in PROTECTED_PATHS:
        try:
            g = guard.resolve()
        except OSError:
            continue
        if rp == g or g in rp.parents or rp in g.parents:
            return True
    return False


def _reap_orphans() -> List[str]:
    """SIGTERM, then SIGKILL, leftover simulator processes. Returns what died."""
    killed: List[str] = []
    for pattern in ORPHAN_PATTERNS:
        try:
            out = subprocess.run(["pgrep", "-f", pattern],
                                 capture_output=True, text=True, timeout=5)
        except (subprocess.SubprocessError, FileNotFoundError):
            continue
        for pid_s in out.stdout.split():
            try:
                pid = int(pid_s)
            except ValueError:
                continue
            if pid == os.getpid():
                continue
            for sig in (signal.SIGTERM, signal.SIGKILL):
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    break                 # already gone
                except PermissionError:
                    break                 # not ours -> leave it alone
                time.sleep(0.4)
                try:
                    os.kill(pid, 0)
                except ProcessLookupError:
                    break
            killed.append(f"{pattern}:{pid}")
    return killed


def _clean_scratch() -> List[str]:
    """Remove Gazebo/JVM scratch only. Refuses symlinks, files owned by someone
    else, and anything inside a protected path."""
    removed: List[str] = []
    for pattern in SCRATCH_GLOBS:
        for hit in glob.glob(pattern):
            p = Path(hit)
            if p.is_symlink() or _is_protected(p):
                continue
            try:
                if p.stat().st_uid != os.getuid():
                    continue              # never force-remove another user's files
                if p.is_dir():
                    shutil.rmtree(p, ignore_errors=True)
                else:
                    p.unlink()
                removed.append(str(p))
            except OSError:
                continue
    return removed


# --------------------------------------------------------------------------- #
# Mission state
# --------------------------------------------------------------------------- #

class MissionState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.proc: Optional[subprocess.Popen] = None
        self.log: List[str] = []
        self.waypoints: List[dict] = []
        self.llm_status = "NO MISSION PLAN LOADED"
        self.mission_id = ""          # id of the plan currently dispatched to the executor
        self.mission_name = ""        # what the planning agent called this job
        self.active_cmd = -1
        self.total_cmd = 0

    @property
    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def append(self, line: str) -> None:
        with self.lock:
            self.log.append(line.rstrip())
            if len(self.log) > LOG_LINES:
                self.log = self.log[-LOG_LINES:]


MISSION = MissionState()


def _enu_to_latlon(east: float, north: float) -> tuple:
    lat = HOME_LAT + (north / 111_320.0)
    lon = HOME_LON + (east / (111_320.0 * math.cos(math.radians(HOME_LAT))))
    return round(lat, 6), round(lon, 6)


def _latest_run_dir() -> Optional[Path]:
    runs = PIPELINE_ROOT / "runs"
    if not runs.is_dir():
        return None
    dirs = sorted((d for d in runs.iterdir() if d.is_dir()), key=lambda d: d.name)
    return dirs[-1] if dirs else None


def _load_waypoints() -> List[dict]:
    """Turn the newest compiled_commands.json into UI/map waypoints."""
    run_dir = _latest_run_dir()
    if not run_dir:
        return []
    f = run_dir / "compiled_commands.json"
    if not f.is_file():
        return []
    try:
        cmds = json.loads(f.read_text(encoding="utf-8")).get("commands", [])
    except (json.JSONDecodeError, OSError):
        return []

    ACTIONS = {
        "Takeoff": "Climb / Hold", "Goto": "Transit, Record",
        "Hold": "Hold, Pan 360", "Land": "Descend / Land",
        "ReturnToLaunch": "Return Home", "Arm": "Arm", "Disarm": "Disarm",
    }

    positional: List[dict] = []
    x = y = 0.0
    for ci, c in enumerate(cmds):
        kind = c.get("cmd")
        if kind == "Takeoff":
            alt, speed = float(c.get("alt", 0.0)), 0.0
        elif kind == "Goto":
            x, y = float(c.get("x", 0.0)), float(c.get("y", 0.0))
            alt, speed = float(c.get("alt", 0.0)), float(c.get("speed_mps", 0.0))
        else:
            continue
        lat, lon = _enu_to_latlon(x, y)
        positional.append({
            "cmd_index": ci, "x": x, "y": y, "alt": alt,
            "lat": lat, "lon": lon, "speed": round(speed * 3.6, 1),
            "action": ACTIONS.get(kind, kind),
        })

    # Energy remaining is a PROJECTION along path length, not a measurement.
    total = 0.0
    prev = (0.0, 0.0)
    for p in positional:
        total += math.dist(prev, (p["x"], p["y"]))
        prev = (p["x"], p["y"])
    cum = 0.0
    prev = (0.0, 0.0)
    n = len(positional)
    for i, p in enumerate(positional):
        cum += math.dist(prev, (p["x"], p["y"]))
        prev = (p["x"], p["y"])
        p["idx"] = i + 1
        p["total"] = n
        p["energy"] = int(round(100 - 40 * (cum / total if total else 0)))
        p["active"] = False
    return positional


def _mark_active() -> None:
    """Highlight the waypoint matching the executor's current command index."""
    for w in MISSION.waypoints:
        w["active"] = (w["cmd_index"] == MISSION.active_cmd)


_PROGRESS = re.compile(r"\[(\d+)/(\d+)\]")


# --------------------------------------------------------------------------- #
# ws_swarm LLM planner
#
# Natural language -> ws_swarm mission_interface (llm_client + validator) -> a
# bounds-checked MissionPlan, published on /mission_plan for the swarm stack to execute.
# The validator is the trust boundary: raw model output is never executed, and an
# out-of-bounds plan is clamped (or rejected loudly) before it reaches a vehicle.
# --------------------------------------------------------------------------- #

_LLM = {"client": None}

# Serialise stack start/stop.
#
# stack_start() reaps orphans FIRST, so two overlapping calls are destructive: the second
# call's reap kills the stack the first one is still bringing up. That shows up as the whole
# swarm being SIGINT-ed seconds after launch ("mission is not executing", nodes vanish).
# One lock, plus an explicit busy reply, makes a double POWER press harmless.
_STACK_LOCK = threading.Lock()
_STACK_BUSY = {"op": None}

# Target images uploaded by the operator, classified once on upload. NOTHING is armed here:
# these only describe what the agents may hunt IF a mission later tells them to. The order is
# always  upload -> LLM plan -> arm agents.
TARGETS: List[dict] = []
_YOLO = {"model": None}


def _classify_target(data_url: str) -> dict:
    """Identify the dominant object in an uploaded image, so the operator's picture becomes
    a detector class the agents can actually acquire."""
    import base64
    raw = data_url.split(",", 1)[1] if "," in data_url else data_url
    img_bytes = base64.b64decode(raw)
    try:
        import cv2
        import numpy as np
        from ultralytics import YOLO
    except Exception as e:
        return {"ok": False, "detail": f"vision stack unavailable: {e}"}

    img = cv2.imdecode(np.frombuffer(img_bytes, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return {"ok": False, "detail": "could not decode image"}
    if _YOLO["model"] is None:
        _YOLO["model"] = YOLO(str(DETECT_MODEL))
    model = _YOLO["model"]
    r = model(img, imgsz=640, conf=0.25, verbose=False)[0]
    if not len(r.boxes):
        return {"ok": False, "detail": "no recognisable object in the image"}
    best = max(r.boxes, key=lambda b: float(b.conf[0]))
    return {"ok": True,
            "label": model.names[int(best.cls[0])],
            "conf": round(float(best.conf[0]), 3)}


# --------------------------------------------------------------------------- #
# Plan normalisation — the robustness boundary.
#
# An LLM will not always name a primitive the way the executor spells it. It may say
# "chase", "pursue", "shadow", "sweep", "patrol", "circle", "return_to_home". Relying on
# the model to emit an exact vocabulary is brittle: in the field there is nobody to edit a
# prompt when it says "intercept" instead of "follow". So every plan is normalised here
# BEFORE it is published: synonyms are mapped onto the executor's real primitives, and
# targets are recovered from wherever the model put them.
#
# Anything that still cannot be mapped is left untouched — the coordinator degrades it to
# `hold`, which is safe — and is reported honestly in the reply.
# --------------------------------------------------------------------------- #

_PRIMITIVE_SYNONYMS: Dict[str, str] = {
    # pursue an object -> follow (perception layer performs it)
    "follow": "follow", "track": "follow", "chase": "follow", "pursue": "follow",
    "shadow": "follow", "tail": "follow", "intercept": "follow", "hunt": "follow",
    "trail": "follow", "engage": "follow", "lock": "follow", "monitor_target": "follow",
    # area coverage -> survey
    "survey": "survey", "scan": "survey", "sweep": "survey", "patrol": "survey",
    "search": "survey", "cover": "survey", "map": "survey", "mapping": "survey",
    "explore": "survey", "reconnaissance": "survey", "recon": "survey", "grid": "survey",
    "inspect_area": "survey",
    # circular / expanding patterns
    "orbit": "orbit", "circle": "orbit", "loiter": "orbit", "encircle": "orbit",
    "circuit": "orbit", "spiral": "spiral", "expand": "spiral", "outward_spiral": "spiral",
    # point to point
    "goto": "goto", "go_to": "goto", "move": "goto", "fly": "goto", "fly_to": "goto",
    "transit": "goto", "navigate": "navigate", "proceed": "goto", "waypoint": "goto",
    "relocate": "goto", "advance": "goto",
    # station keeping
    "hold": "hold", "hover": "hold", "station": "hold", "standby": "hold",
    "hold_position": "hold", "maintain": "hold", "observe": "hold", "watch": "hold",
    # start / finish
    "takeoff": "takeoff", "take_off": "takeoff", "launch": "takeoff",
    "liftoff": "takeoff", "ascend": "takeoff", "climb": "takeoff",
    "rtl": "rtl", "return": "rtl", "return_home": "rtl", "return_to_home": "rtl",
    "go_home": "rtl", "come_home": "rtl", "recall": "rtl",
    "land": "land", "descend": "land", "touchdown": "land", "landing": "land",
    # formation + timing
    "formation": "formation", "form": "formation", "formup": "formation",
    "form_up": "formation", "regroup": "formation", "assemble": "formation",
    "wait": "wait", "pause": "wait", "delay": "wait", "dwell": "wait", "loiter_time": "wait",
}


def _canonical_primitive(raw: str) -> Optional[str]:
    """Map whatever the model wrote onto an executor primitive, or None if unknown."""
    if not raw:
        return None
    key = str(raw).strip().lower().replace("-", "_").replace(" ", "_")
    if key in _PRIMITIVE_SYNONYMS:
        return _PRIMITIVE_SYNONYMS[key]
    # substring match catches things like "survey_area", "follow_the_person"
    for word, canon in _PRIMITIVE_SYNONYMS.items():
        if word in key:
            return canon
    return None


def _infer_target(node: dict, phase_name: str, uploaded: List[str]) -> Optional[str]:
    """Find the object this node is about: explicit param first, then any uploaded label
    mentioned in the params or the phase/assignment name."""
    params = node.get("params") or {}
    for k in ("target", "object", "subject", "class", "target_class"):
        v = params.get(k)
        if v:
            return str(v)
    blob = f"{phase_name} {node.get('name', '')} {json.dumps(params)}".lower()
    for label in uploaded:
        if label.lower() in blob:
            return label
    return None


_AREA_PATTERNS = ("survey", "orbit", "spiral")
_SURVEY_KEYS = ("center", "width", "height", "spacing")
# Primitives that are explicit non-hunting work. A drone given one of these was tasked to
# fly, not to look for anything, so it must never be armed by a phase-level name match.
_NON_FOLLOW = ("formation", "goto", "navigate", "hold", "wait", "land", "rtl", "takeoff")


# Geometry the executor reads with p['key'] - a missing one raises KeyError inside its plan
# callback, which discards the WHOLE mission before it logs anything: drones end up armed but
# never tasked, and nothing anywhere says why. Defaults are a 60x60 m box about the origin.
_SURVEY_DEFAULTS = {"center": [0.0, 0.0], "width": 60.0, "height": 60.0, "spacing": 10.0}


def _takeoff_altitude(plan: dict, fallback: float = 10.0) -> float:
    """The mission's takeoff altitude - what a phase that omits one should inherit."""
    for ph in plan.get("phases", []) or []:
        if ph.get("primitive") == "takeoff":
            return float((ph.get("params") or {}).get("altitude", fallback))
    return fallback


def _complete_survey_params(plan: dict, cruise_alt: float) -> List[str]:
    """Fill in missing survey/orbit/spiral geometry in place. -> notes on what was assumed.

    A planner that names an area but omits one field would otherwise kill the mission
    silently. Assuming a value is strictly better than discarding the plan, and every
    assumption is reported back to the operator rather than hidden.
    """
    notes: List[str] = []

    def fix(node: dict, where: str) -> None:
        prim = node.get("primitive")
        if prim not in _AREA_PATTERNS:
            return
        params = node.setdefault("params", {})
        needed = dict(_SURVEY_DEFAULTS) if prim == "survey" else {"center": [0.0, 0.0]}
        for key, default in needed.items():
            if key not in params:
                params[key] = default
                notes.append(f"{where} {prim}: assumed {key}={default}")
        if "altitude" not in params:
            params["altitude"] = cruise_alt
            notes.append(f"{where} {prim}: assumed altitude={cruise_alt}")
        if prim in ("orbit", "spiral") and not (params.get("radius") or params.get("start_radius")):
            params["radius"] = 15.0
            notes.append(f"{where} {prim}: assumed radius=15.0")

    for ph in plan.get("phases", []) or []:
        name = ph.get("name", "phase")
        fix(ph, name)
        for a in ph.get("assignments", []) or []:
            fix(a, f"{name}/{a.get('drone', '?')}")
    return notes


def _is_search_node(a: dict) -> bool:
    """True when this phase or assignment describes an area to cover, whatever it is named.

    Asked to "search an area and follow what you find", a planner usually writes ONE `follow`
    assignment and hangs the search area on it - centre, width, height and spacing all present
    and unambiguous. Keying off the primitive name alone threw that geometry away and the
    drone was left holding, so it never covered the area and never saw the target to follow.
    """
    if a.get("primitive") in _AREA_PATTERNS:
        return True
    params = a.get("params") or {}
    return all(k in params for k in _SURVEY_KEYS)


def _pattern_waypoints(primitive: str, p: dict) -> List[Tuple[float, float]]:
    """Ground track for one area pattern, in the geometry the executor already uses.

    survey -> lawnmower lanes across width x height, rotated by heading.
    orbit  -> a closed ring of radius r.  spiral -> a ring whose radius grows each turn.
    """
    cx, cy = float(p["center"][0]), float(p["center"][1])
    pts: List[Tuple[float, float]] = []

    if primitive == "survey":
        w, h, sp = float(p["width"]), float(p["height"]), float(p["spacing"])
        th = math.radians(float(p.get("heading", 0.0)))
        cos_t, sin_t = math.cos(th), math.sin(th)
        for i in range(max(1, int(h / sp) + 1)):
            oy = -h / 2.0 + min(i * sp, h)
            ends = [(-w / 2.0, oy), (w / 2.0, oy)]
            if i % 2:                           # alternate direction => lawnmower
                ends.reverse()
            for ex, ey in ends:
                pts.append((round(cx + ex * cos_t - ey * sin_t, 3),
                            round(cy + ex * sin_t + ey * cos_t, 3)))
        return pts

    turns = max(1, int(p.get("turns", 1)))
    n = max(1, int(p.get("points_per_turn", 12)))
    r0 = float(p.get("radius", p.get("start_radius", 10.0)))
    growth = float(p.get("growth", 0.0)) if primitive == "spiral" else 0.0
    for t in range(turns):
        for k in range(n):
            frac = k / n
            r = r0 + growth * (t + frac)
            a = 2.0 * math.pi * frac
            pts.append((round(cx + r * math.cos(a), 3), round(cy + r * math.sin(a), 3)))
    if primitive == "orbit":                    # close the ring
        pts.append((round(cx + r0, 3), round(cy, 3)))
    return pts


def _expand_assignment_patterns(plan: dict) -> Tuple[dict, int]:
    """Turn PER-DRONE area patterns into the point primitives the executor understands.

    A `survey` written inside `assignments` - how a plan says "drone 1 searches the north
    sector, drone 2 the south" - reached the executor untouched, and its action table has no
    entry for `survey`, so it degraded to `hold` and the drone never searched at all. Only
    assignment-level patterns are rewritten here; phase-level ones are already expanded
    downstream and are deliberately left alone.

    Each drone's pattern becomes its own waypoint list, and the lists are zipped into one
    phase per leg so the drones fly their sectors AT THE SAME TIME rather than in turn. A
    drone whose list runs out repeats its last waypoint, which holds it in place instead of
    dropping it out of the `all_arrived` gate and stalling the phase for everyone.

    Returns (plan_for_execution, legs_added); the plan is unchanged when there is nothing
    to expand, and any pattern with unusable geometry is passed through rather than raising.
    """
    out_phases: List[dict] = []
    legs_added = 0

    # A search assignment rarely restates the altitude, so inherit the mission's takeoff
    # altitude instead of inventing one that would sink the drone mid-flight.
    cruise_alt = 10.0
    for ph in plan.get("phases", []) or []:
        if ph.get("primitive") == "takeoff":
            cruise_alt = float((ph.get("params") or {}).get("altitude", cruise_alt))
            break

    for ph in plan.get("phases", []) or []:
        assigns = ph.get("assignments") or []

        # PHASE-LEVEL search: a primitive the executor cannot fly - typically "follow" - that
        # nonetheless carries a whole area. This is what a single-drone "search and follow the
        # person" produces, and it used to hold on the spot. Patterns the executor DOES expand
        # itself (survey/orbit/spiral) are deliberately left to it.
        if (not assigns and _is_search_node(ph)
                and ph.get("primitive") not in _AREA_PATTERNS):
            try:
                params = ph.get("params") or {}
                alt = float(params.get("altitude", cruise_alt))
                speed = {"speed": params["speed"]} if params.get("speed") else {}
                pts = _pattern_waypoints("survey", params)
            except (KeyError, TypeError, ValueError, IndexError, ZeroDivisionError):
                out_phases.append(ph)
                continue
            base = ph.get("name", "search")
            for i, (x, y) in enumerate(pts):
                out_phases.append({
                    "name": f"{base}_leg{i}",
                    "primitive": "goto",
                    "params": {**speed, "target": [x, y, alt]},
                    "drones": ph.get("drones"),
                    "advance_when": ph.get("advance_when", "all_arrived"),
                    "timeout": ph.get("timeout", 60.0),
                })
            legs_added += len(pts)
            continue

        if not any(_is_search_node(a) for a in assigns):
            out_phases.append(ph)
            continue

        routes: Dict[str, List[dict]] = {}
        try:
            for a in assigns:
                drone = a.get("drone")
                if not drone:
                    continue
                prim, params = a.get("primitive"), (a.get("params") or {})
                if not _is_search_node(a):
                    routes[drone] = [a]         # non-search work stands for the whole phase
                    continue
                # Rings carry radius/turns; anything else with a full area is a lawnmower.
                kind = prim if prim in ("orbit", "spiral") else "survey"
                alt = float(params.get("altitude", cruise_alt))
                speed = {"speed": params["speed"]} if params.get("speed") else {}
                routes[drone] = [
                    {"drone": drone, "primitive": "goto",
                     "params": {**speed, "target": [x, y, alt]}}
                    for x, y in _pattern_waypoints(kind, params)
                ]
        except (KeyError, TypeError, ValueError, IndexError, ZeroDivisionError):
            out_phases.append(ph)               # malformed geometry: leave the phase as written
            continue

        if not routes:
            out_phases.append(ph)
            continue

        base = ph.get("name", "search")
        legs = max(len(r) for r in routes.values())
        for i in range(legs):
            out_phases.append({
                "name": f"{base}_leg{i}",
                "assignments": [routes[d][min(i, len(routes[d]) - 1)] for d in sorted(routes)],
                "advance_when": ph.get("advance_when", "all_arrived"),
                "timeout": ph.get("timeout", 60.0),
            })
        legs_added += legs

    return ({**plan, "phases": out_phases}, legs_added) if legs_added else (plan, 0)


def _normalise_plan(plan: dict) -> tuple:
    """Rewrite a plan into the executor's vocabulary. Returns (plan, notes)."""
    notes: List[str] = []
    uploaded = sorted({t["label"] for t in TARGETS})

    def fix_node(node: dict, phase_name: str) -> None:
        raw = node.get("primitive")
        canon = _canonical_primitive(raw)
        if canon and canon != raw:
            notes.append(f'"{raw}" -> {canon}')
            node["primitive"] = canon
        elif not canon and raw:
            notes.append(f'"{raw}" unknown - executor will hold')
        # a follow node must carry the object it is following
        if node.get("primitive") == "follow":
            params = node.setdefault("params", {})
            if not params.get("target"):
                inferred = _infer_target(node, phase_name, uploaded)
                if inferred:
                    params["target"] = inferred
                    notes.append(f'target "{inferred}" recovered')

    for ph in plan.get("phases", []) or []:
        name = ph.get("name", "")
        if ph.get("primitive"):
            fix_node(ph, name)
        for a in ph.get("assignments", []) or []:
            fix_node(a, name)
    return plan, notes


def _plan_target_map(plan: dict) -> Dict[str, str]:
    """drone -> the target class THAT drone was tasked with.

    The planner emits follow/track work either as a per-drone "assignments" list or as a
    phase scoped with "drones". Each carries its own {"target": "<class>"}, so px4_1 hunting
    the person and px4_2 hunting the car stay separate. Previously every armed drone was
    given the union of all uploaded labels, so all three chased everything.
    """
    want: Dict[str, str] = {}

    def is_follow(prim: str, name: str = "") -> bool:
        blob = f"{prim or ''} {name or ''}".lower()
        return "follow" in blob or "track" in blob or "hunt" in blob

    uploaded = sorted({t["label"] for t in TARGETS})

    for ph in plan.get("phases", []) or []:
        pname = ph.get("name", "")
        # per-drone assignments (the parallel form)
        for a in ph.get("assignments", []) or []:
            d = a.get("drone")
            # Judge an assignment by ITS OWN primitive and name. Falling back to the phase name
            # here made every drone in a phase called "search_and_track" look like a hunter,
            # including the ones explicitly assigned a formation.
            if d and is_follow(a.get("primitive"), a.get("name", "")):
                # params first, then anything recoverable from the name ("follow_person")
                tgt = (a.get("params") or {}).get("target") or _infer_target(a, pname, uploaded)
                want[d] = str(tgt) if tgt else want.get(d, "")
        # phase-level follow, scoped to some or all drones
        if is_follow(ph.get("primitive"), pname):
            tgt = (ph.get("params") or {}).get("target") or _infer_target(ph, pname, uploaded)
            # Per-drone assignments are AUTHORITATIVE about who is hunting. Falling through to
            # every drone here armed the whole swarm off a phase merely NAMED "search_and_track",
            # so escorts explicitly given formation work switched their detectors on and chased
            # the target too. A drone told to fly a formation is not hunting.
            assigns = ph.get("assignments") or []
            if assigns:
                scope = [a["drone"] for a in assigns
                         if a.get("drone") and a.get("primitive") not in _NON_FOLLOW]
            else:
                scope = ph.get("drones") or _drone_ids()
            for d in scope:
                if tgt:
                    want.setdefault(d, str(tgt))
                else:
                    want.setdefault(d, "")        # follow, target genuinely unnamed
    return want


def _arm_messages(plan: dict) -> Tuple[List[dict], Dict[str, str]]:
    """Messages that arm detection/follow for the drones this plan tasks — each with ITS target.

    Pure: it builds messages and publishes nothing, so arming can ride in the same batch as
    the plan itself rather than paying its own discovery round per drone.

    This is the ONLY place perception is switched on. Until it runs every detector sits on
    the IDLE_CLASS sentinel and publishes nothing, so a drone can never start chasing a
    person that merely walks past the camera.

    Target resolution per drone:
      1. the target named in that drone's follow assignment, if the operator uploaded a
         matching image (the uploaded label is authoritative — it is what the model can
         actually detect),
      2. otherwise the union of uploaded labels (single-target missions).
    """
    if not TARGETS:
        return [], {}
    uploaded = sorted({t["label"] for t in TARGETS})
    all_classes = ",".join(uploaded)

    want = _plan_target_map(plan)
    if not want:
        # No follow phase at all -> nothing to arm; the drones just fly the plan.
        return [], {}

    messages, classes_by_drone = [], {}
    for d in sorted(want):
        asked = (want[d] or "").strip().lower()
        # match the requested word against what was actually uploaded/classified
        match = next((u for u in uploaded if u in asked or asked in u), None) if asked else None
        classes = match or all_classes
        classes_by_drone[d] = classes
        messages.append({"topic": f"/{d}/set_target", "type": "std_msgs/msg/String",
                         "fields": {"data": classes}})
        messages.append({"topic": f"/{d}/follow_reset", "type": "std_msgs/msg/String",
                         "fields": {"data": "mission arm"}})
    return messages, classes_by_drone


def _armed_labels(classes_by_drone: Dict[str, str], results: Dict[str, bool]) -> List[str]:
    """"px4_1:person" for each drone whose set_target actually reached a live detector."""
    return [f"{d}:{c}" for d, c in sorted(classes_by_drone.items())
            if results.get(f"/{d}/set_target")]


def _arm_agents(plan: dict) -> List[str]:
    """Arm perception in its own batch, for callers outside the mission path."""
    messages, classes_by_drone = _arm_messages(plan)
    return _armed_labels(classes_by_drone, _ros_publish_many(messages)) if messages else []


_BATCH_SNIPPET = r'''
"""Publish a list of ROS 2 messages from ONE process, then exit.

Every short-lived publisher pays the same fixed bill: interpreter start, rclpy init, and
DDS discovery of the whole graph. Paying it once for N messages instead of N times is
where the latency goes. The publishers are also created together, so their endpoint
matching overlaps rather than running back to back.
"""
import importlib
import json
import sys
import time

import rclpy
from rclpy.node import Node

specs = json.load(open(sys.argv[1]))
wait_s = float(sys.argv[2])

rclpy.init()
node = Node("aegis_batch_publisher")

pubs = []
for spec in specs:
    pkg, _, cls = spec["type"].replace("/msg/", "/").partition("/")
    msg_cls = getattr(importlib.import_module(pkg + ".msg"), cls)
    msg = msg_cls()
    for key, value in spec["fields"].items():
        setattr(msg, key, value)
    pubs.append((spec["topic"], node.create_publisher(msg_cls, spec["topic"], 10), msg))

# ONE shared discovery window for every topic, instead of one window per message.
deadline = time.time() + wait_s
while time.time() < deadline and any(p.get_subscription_count() == 0 for _, p, _ in pubs):
    rclpy.spin_once(node, timeout_sec=0.05)

# These topics are latch-less, so a single publish can still race a subscriber that is
# mid-match. Repeats are idempotent: the coordinator keys on mission_id, detectors on the
# class string. Order within a round is list order, so the plan lands before the arming.
for _ in range(3):
    for _, pub, msg in pubs:
        pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.1)

print(json.dumps({t: p.get_subscription_count() > 0 for t, p, _ in pubs}))
node.destroy_node()
rclpy.shutdown()
'''


def _ros_publish_many(messages: List[dict], wait_s: float = 5.0) -> Dict[str, bool]:
    """Publish every message from ONE short-lived rclpy process.

    Each message is {"topic", "type", "fields"}. Returns {topic: matched_a_subscriber} so
    the caller can report an undelivered topic honestly instead of assuming a dispatch.
    Running it as a subprocess keeps this supervisor rclpy-free, so a ROS-side crash can
    never take the API down.
    """
    if not messages:
        return {}
    spec_path = script_path = None
    try:
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as fh:
            json.dump(messages, fh)
            spec_path = fh.name
        with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as fh:
            fh.write(_BATCH_SNIPPET)
            script_path = fh.name

        cmd = (f"source {ROS_SETUP} && source {WS_SWARM_SETUP} && "
               f"export FASTRTPS_DEFAULT_PROFILES_FILE={DDS_PROFILE} && "
               f"exec python3 {script_path} {spec_path} {wait_s}")
        r = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True,
                           timeout=wait_s + 25)
        if r.returncode != 0:
            return {}
        return json.loads((r.stdout or "{}").strip().splitlines()[-1])
    except Exception:
        return {}
    finally:
        for p in (spec_path, script_path):
            if p:
                try:
                    os.unlink(p)
                except OSError:
                    pass


def _ros_publish(topic: str, msg_type: str, fields: dict) -> bool:
    """One-shot publish. Thin wrapper so single-message callers read the same as before."""
    return _ros_publish_many(
        [{"topic": topic, "type": msg_type, "fields": fields}], wait_s=3.0).get(topic, False)


def _ws_swarm_python() -> None:
    """Make ws_swarm's python packages importable in this process (idempotent)."""
    import sys
    for p in (WS_SWARM / "src" / "mission_interface",
              WS_SWARM / "install" / "mission_interface" / "lib" / "python3.10" / "site-packages"):
        sp = str(p)
        if p.exists() and sp not in sys.path:
            sys.path.insert(0, sp)


def plan_with_ws_swarm(prompt: str) -> dict:
    """Plan + validate a natural-language mission. Returns a JSON-able result dict."""
    _ws_swarm_python()
    try:
        from mission_interface.llm_client import LLMClient
        from mission_interface import validator
    except Exception as e:                     # planner not installed / not built
        return {"ok": False, "detail": f"LLM planner unavailable: {e}"}

    try:
        if _LLM["client"] is None:
            _LLM["client"] = LLMClient(default_altitude=4.0, drones=_drone_ids())
        plan, unsupported = _LLM["client"].plan(prompt)

        # Map whatever vocabulary the model used onto the executor's primitives first, so
        # validation and dispatch both see a canonical plan. This is what lets an arbitrary
        # prompt execute without anyone hand-tuning the wording.
        plan, norm_notes = _normalise_plan(plan)

        plan = validator.normalize_plan(plan)
        adjustments = validator.clamp_plan(plan)
        ok, errors = validator.validate_plan(plan)

        # Degrade, don't discard: feed the errors back once and let the planner repair.
        if not ok:
            try:
                fixed, unsupported = _LLM["client"].repair(prompt, plan, errors)
                fixed = validator.normalize_plan(fixed)
                validator.clamp_plan(fixed)
                ok, errors = validator.validate_plan(fixed)
                if ok:
                    plan = fixed
            except Exception:
                pass
        if not ok:
            return {"ok": False, "detail": "; ".join(errors[:3]) or "failed validation"}

        phases = plan.get("phases", [])
        summary = " -> ".join(p.get("name", p.get("primitive", "?")) for p in phases) or "empty plan"
        reply = f"Plan accepted: {summary}."
        if adjustments:
            reply += f" ({len(adjustments)} value(s) clamped to safe limits)"
        if unsupported:
            reply += f" Unsupported: {', '.join(unsupported)}."

        # Step 3 of upload -> plan -> ARM. Only now may the agents detect/track/follow, and
        # only the drones this plan actually tasks. The plan and every drone's arming go out
        # in ONE batch: same order, same guarantees, but a single DDS discovery round for the
        # whole mission instead of one per message. Identical for single and swarm - the
        # drone count only changes the length of the list.
        # Per-drone area patterns become concrete waypoints HERE, so the executor only ever
        # receives primitives it can act on. The console keeps showing the compact plan; what
        # flies is this expansion of it.
        # An incomplete area would raise KeyError inside the executor and silently bin the
        # whole mission, so any missing geometry is filled and reported before dispatch.
        assumed = _complete_survey_params(plan, _takeoff_altitude(plan))
        if assumed:
            reply += f" Assumed: {'; '.join(assumed[:3])}."

        exec_plan, legs = _expand_assignment_patterns(plan)
        plan_msg = _plan_message(exec_plan, unsupported)
        MISSION.mission_id = plan_msg["fields"]["mission_id"]
        MISSION.mission_name = _mission_name(plan)
        if legs:
            reply += f" Search expanded to {legs} legs."

        arm_msgs, arm_classes = _arm_messages(plan)
        results = _ros_publish_many([plan_msg] + arm_msgs)

        published = results.get("/mission_plan", False)
        if not published:
            reply += " (stack offline — plan not dispatched)"
        armed = _armed_labels(arm_classes, results) if published else []
        if armed:
            reply += f" Agents armed: {', '.join(armed)} (hunting {', '.join(sorted({t['label'] for t in TARGETS}))})."
        elif TARGETS and published:
            reply += " No follow phase in this plan — agents stay passive."

        return {"ok": True, "summary": summary, "reply": reply,
                "plan": plan, "unsupported": list(unsupported or []),
                "adjustments": adjustments, "published": published, "armed": armed}
    except Exception as e:
        return {"ok": False, "detail": str(e)[:300]}


def _plan_message(plan: dict, unsupported) -> dict:
    """The /mission_plan message for a validated plan.

    plan_json stays canonical JSON TEXT and is never routed through a YAML parser - the
    `ros2 topic pub` CLI did exactly that and silently stripped the quotes, leaving the
    executor unable to read a plan it had apparently received.
    """
    return {
        "topic": "/mission_plan",
        "type": "swarm_msgs/msg/MissionPlan",
        "fields": {
            "mission_id": f"gcs-{int(time.time())}",
            "plan_json": json.dumps(plan),
            "unsupported": list(unsupported or []),
        },
    }




# --------------------------------------------------------------------------- #
# API
# --------------------------------------------------------------------------- #

@asynccontextmanager
async def _lifespan(_app: FastAPI):
    yield
    # never leave orphaned SITL/gazebo processes behind on API shutdown
    for name in reversed(START_ORDER):
        SERVICES[name].stop()


app = FastAPI(title="AEGIS Control API", lifespan=_lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],          # LAN-local GCS; tighten if exposed
    allow_methods=["*"],
    allow_headers=["*"],
)


class MissionRequest(BaseModel):
    prompt: str


def _cap_logs() -> None:
    """Truncate any service log that has run away. Cheap, and it guarantees a
    misbehaving child can never fill the disk."""
    for s in SERVICES.values():
        p = s.log_path
        if not p or not p.is_file():
            continue
        try:
            if p.stat().st_size > MAX_LOG_BYTES:
                with p.open("r+", encoding="utf-8", errors="replace") as fh:
                    fh.seek(0)
                    fh.truncate()
                    fh.write(f"[control_api] log exceeded "
                             f"{MAX_LOG_BYTES // 1048576} MB and was truncated\n")
        except OSError:
            continue


# A service is "up" if THIS api spawned it, or if its process is alive at all. Without the
# second check, restarting the control API makes a perfectly healthy simulation report as
# stopped, because the new instance has no handle on the old children.
_LIVENESS_PATTERNS: Dict[str, str] = {
    "agent": "MicroXRCEAgent udp4",
    "sitl": "gzserver",
    "rosbridge": "rosbridge_websocket",
    "telemetry": "telemetry_node.py",
    "video": "video_streamer.py",
    "swarm": "coordinator_node",
}


def _externally_alive(name: str) -> bool:
    pat = _LIVENESS_PATTERNS.get(name)
    if not pat:
        return False
    try:
        r = subprocess.run(["pgrep", "-f", pat], capture_output=True, text=True, timeout=4)
        return bool(r.stdout.strip())
    except (subprocess.SubprocessError, FileNotFoundError):
        return False


def _autonomy_state() -> str:
    """What level of autonomy is actually in effect right now.

    Was a hardcoded "SUPERVISED" with nothing behind it. Derived instead from real state:
      AUTONOMOUS  decentralised coordination is on (agents self-assign and self-heal), or
                  perception is armed and hunting without further operator input
      SUPERVISED  a mission is loaded/running with the operator in the loop
      MANUAL      nothing tasked; the operator drives
    """
    running = bool(SERVICES["sitl"].running or _externally_alive("sitl"))
    if not running:
        return "STANDBY"
    if SIM_CONFIG.get("agentic"):
        return "AUTONOMOUS"
    if TARGETS and MISSION.llm_status.startswith(("LLM-VALIDATED", "MISSION")):
        return "AUTONOMOUS"
    if MISSION.llm_status.startswith(("LLM-VALIDATED", "MISSION")):
        return "SUPERVISED"
    return "MANUAL"


@app.get("/api/stack/status")
def stack_status() -> dict:
    """Console-facing status.

    Shape matters: the dashboard reads `services` as a LIST of {name, running, detail} and
    `running` as the overall flag. Returning a name-keyed dict (the old shape) made every
    indicator read "STOPPED" no matter what the simulation was doing.
    """
    _cap_logs()
    services = []
    for n, s in SERVICES.items():
        alive = s.running or _externally_alive(n)
        services.append({
            "name": n,
            "running": alive,
            "adopted": (not s.running) and alive,   # alive, but not started by this API
            "pid": s.proc.pid if s.running and s.proc else None,
            "detail": s.detail,
            "log": str(s.log_path) if s.log_path else None,
        })
    sitl_up = any(x["name"] == "sitl" and x["running"] for x in services)
    return {
        "services": services,
        "running": sitl_up,                  # the sim is "on" when Gazebo/SITL is up
        "mode": SIM_CONFIG.get("mode"),
        "world": SIM_CONFIG.get("world"),
        "drones": SIM_CONFIG.get("drones"),
        "autonomy": _autonomy_state(),
        "gps_denied": bool(SIM_CONFIG.get("gps_denied")),
        "agentic": bool(SIM_CONFIG.get("agentic")),
        "targets": [{k: t[k] for k in ("name", "label", "conf")} for t in TARGETS],
    }


@app.get("/api/stack/log/{name}")
def stack_log(name: str, lines: int = 60) -> dict:
    """Tail a service log so launch failures are visible from the dashboard."""
    if name not in SERVICES:
        return {"ok": False, "detail": f"unknown service '{name}'"}
    return {"ok": True, "name": name, "tail": SERVICES[name].tail(lines)}


@app.post("/api/stack/cleanup")
def stack_cleanup() -> dict:
    """Reap orphaned sim processes + clear Gazebo scratch. Non-destructive:
    never touches models, build output, or the serial hardware agent."""
    killed = _reap_orphans()
    removed = _clean_scratch()
    return {"ok": True, "killed": killed, "removed": removed}


def _drone_ids() -> List[str]:
    """px4_1..px4_N for the current vehicle count (ws_swarm naming)."""
    return [f"px4_{i}" for i in range(1, int(SIM_CONFIG.get("drones", 1)) + 1)]


def _apply_sim_config() -> None:
    """Rebuild the SITL + swarm commands from the current selection.

    single : PX4's own make target (fast, one vehicle, gzclient GUI).
    swarm  : ws_swarm's sitl_multiple_run.sh (N vehicles in one world) followed by the
             ws_swarm perception/follow stack, so a swarm mission is fully autonomous.
    Both inherit SITL_ENV, which is what keeps Gazebo from hanging on the online model DB.
    """
    mode = SIM_CONFIG.get("mode", "single")
    world = SIM_CONFIG["world"]
    n = int(SIM_CONFIG.get("drones", 1))

    # ONE launcher for both modes — single is just n=1.
    #
    # This is deliberate. `make px4_sitl gazebo-classic_iris` starts PX4 as instance 0, which
    # publishes on BARE topics (/fmu/out/...). Every ws_swarm node subscribes to NAMESPACED
    # topics (/px4_1/fmu/...  — drone_controller builds them from `ns = f'/{drone_id}'`), so
    # with the make target the controller never saw the vehicle and its setpoints never
    # reached PX4: a single drone simply could not be flown. sitl_multiple_run.sh runs
    # `px4 -i N`, giving /px4_N/... for every vehicle, so one code path serves both modes.
    if SWARM_LAUNCHER.exists():
        cmd = (f"cd {PX4_DIR} && exec {SWARM_LAUNCHER} "
               f"-m {SIM_CONFIG['model']} -n {n} -w {world}")
        SERVICES["sitl"].argv = ["bash", "-c", cmd]
        SERVICES["sitl"].detail = (f"{n}x {SIM_CONFIG['model']} / {world} "
                                   f"({'swarm' if n > 1 else 'single'})")
    else:
        # Fallback only if the multi-vehicle launcher is missing from this PX4 checkout.
        target = _sitl_target()
        SERVICES["sitl"].argv = ["bash", "-c", f"exec make px4_sitl {target}"]
        SERVICES["sitl"].detail = f"{SIM_CONFIG['model']} / {world} (single, bare topics)"

    # Perception + follow for swarm missions (detector, target_follow, controllers).
    #
    # IMPORTANT — the stack comes up IDLE. Powering on must never make a drone chase the
    # first person that wanders into frame. Two things enforce that:
    #   target_class:=IDLE_CLASS  the detector only publishes classes in its target set, so a
    #                             sentinel that matches nothing means zero detections. (An
    #                             EMPTY list would mean "no filter" = detect everything.)
    #   search:=true              the follower MAY yaw-scan, but only while parked: it is
    #                             armed on the IDLE sentinel, so with nothing to detect it
    #                             has nothing to scan for and stays put regardless.
    # A mission arms the agents later via /api/mission/run -> _arm_agents(), which sets the
    # real target class per drone. Until then the vehicles fly, but they do not hunt.
    drones = ",".join(_drone_ids())
    swarm_cmd = (
        f"source {ROS_SETUP} && source {WS_SWARM_SETUP} && "
        f"export FASTRTPS_DEFAULT_PROFILES_FILE={DDS_PROFILE} && "
        f"exec ros2 launch sim_bringup stage1_swarm.launch.py drones:={drones} "
        f"perception:=true follow:=true camera:=front search:=true viewer:=false "
        # GPS-denied means the drone has no GPS to route on, so it navigates on the map it
        # is building: nav brings up the A* planner, plan_on_slam points that planner at the
        # rtabmap grid instead of the lidar grid. Both follow gps_denied, so a normal GPS
        # mission launches exactly as it did before.
        f"nav:={str(bool(SIM_CONFIG.get('gps_denied'))).lower()} "
        f"plan_on_slam:={str(bool(SIM_CONFIG.get('gps_denied'))).lower()} "
        f"agentic:={str(bool(SIM_CONFIG.get('agentic'))).lower()} "
        f"slam:={str(bool(SIM_CONFIG.get('gps_denied'))).lower()} "
        # GPS-denied means navigating on SLAM, so open the map view with it. Without this
        # the launch's slam_viz defaulted to false and the operator had SLAM running with
        # no way to see the map it was building.
        f"slam_viz:={str(bool(SIM_CONFIG.get('gps_denied'))).lower()} "
        f"model:={DETECT_MODEL} target_class:={IDLE_CLASS}"
    )
    SERVICES["swarm"].argv = ["bash", "-c", swarm_cmd]
    SERVICES["swarm"].detail = f"{drones} perception+follow"

    # Telemetry + video must consolidate the SAME vehicle set the sim is running, so the
    # console's panels and camera views line up with the mission that was actually started.
    vehicle_env = {"AEGIS_DRONES": str(n), "AEGIS_MODE": str(mode)}
    for svc in ("telemetry", "video"):
        SERVICES[svc].env = {**(SERVICES[svc].env or {}), **vehicle_env}

    # Same DDS transport for EVERY service, or they never discover each other.
    dds = _dds_env()
    for svc in SERVICES:
        SERVICES[svc].env = {**(SERVICES[svc].env or {}), **dds}
    SERVICES["telemetry"].detail = f"/gcs/consolidated_telemetry ({n} vehicle(s))"
    SERVICES["video"].detail = f"MJPEG :8080 ({n} feed(s), raw+annotated)"


def _wait_for_sitl(timeout_s: float) -> bool:
    """Block until Gazebo has actually finished loading the world.

    Heavy worlds take far longer than `empty`, so a fixed sleep either wastes
    time or starts the downstream services before the DDS link exists. Poll for
    a live gzserver instead, and bail out early if SITL dies.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not SERVICES["sitl"].running:
            return False                      # SITL exited -> build/world error
        try:
            out = subprocess.run(["pgrep", "-f", "gzserver"],
                                 capture_output=True, text=True, timeout=5)
            if out.stdout.strip():
                time.sleep(2.0)               # let the world settle
                return True
        except (subprocess.SubprocessError, FileNotFoundError):
            pass
        time.sleep(1.0)
    return False


def _list_movings() -> List[str]:
    """Moving targets we can guarantee, i.e. those whose world exists locally."""
    worlds = set(_list_worlds())
    return ["none"] + [m for m, w in MOVING_WORLDS.items() if w in worlds]


@app.get("/api/sim/config")
def sim_config() -> dict:
    return {**SIM_CONFIG, "target": _sitl_target(),
            "worlds": _list_worlds(), "models": _list_models(),
            "movings": _list_movings()}


@app.post("/api/sim/config")
def set_sim_config(model: Optional[str] = None,
                   world: Optional[str] = None,
                   mode: Optional[str] = None,
                   drones: Optional[int] = None,
                   moving: Optional[str] = None,
                   agentic: Optional[bool] = None) -> dict:
    """Select airframe / world / mission shape. Unknown names are rejected so a typo can
    never produce a make target or launch that hangs. Takes effect on the next start."""
    if model is not None:
        if model not in _list_models():
            return {"ok": False, "detail": f"unknown model '{model}'"}
        SIM_CONFIG["model"] = model
    if world is not None:
        if world not in _list_worlds():
            return {"ok": False, "detail": f"unknown world '{world}'"}
        SIM_CONFIG["world"] = world
    if mode is not None:
        if mode not in ("single", "swarm"):
            return {"ok": False, "detail": f"unknown mode '{mode}'"}
        SIM_CONFIG["mode"] = mode
        # keep the vehicle count consistent with the chosen shape
        if mode == "single":
            SIM_CONFIG["drones"] = 1
        elif int(SIM_CONFIG.get("drones", 1)) < 2:
            SIM_CONFIG["drones"] = 3
    if drones is not None:
        n = max(1, min(5, int(drones)))
        SIM_CONFIG["drones"] = n
        SIM_CONFIG["mode"] = "single" if n == 1 else "swarm"
    if agentic is not None:
        SIM_CONFIG["agentic"] = bool(agentic)
    if moving is not None:
        if moving not in _list_movings():
            return {"ok": False, "detail": f"unknown moving target '{moving}'"}
        SIM_CONFIG["moving"] = moving
        # a moving target implies the world that carries it
        if moving != "none":
            SIM_CONFIG["world"] = MOVING_WORLDS[moving]
    _apply_sim_config()
    return {"ok": True, **sim_config()}


@app.post("/api/stack/start")
def stack_start() -> dict:
    if not _STACK_LOCK.acquire(blocking=False):
        return {**stack_status(), "busy": True,
                "detail": f"already {_STACK_BUSY['op'] or 'working'} — ignoring duplicate request"}
    _STACK_BUSY["op"] = "starting"
    try:
        return _stack_start_locked()
    finally:
        _STACK_BUSY["op"] = None
        _STACK_LOCK.release()


def _stack_start_locked() -> dict:
    # Always start from a clean slate: a surviving gzserver from the previous
    # run is what produces the "not responding" dialog and the lag.
    _reap_orphans()
    _clean_scratch()
    _apply_sim_config()
    time.sleep(1.0)
    for name in _start_order():
        SERVICES[name].start()
        if name == "sitl":
            # Wait for the world to finish loading rather than guessing. This is
            # what makes heavy worlds work without racing the DDS link.
            _wait_for_sitl(SITL_READY_TIMEOUT_S)
        else:
            time.sleep(2.0 if name == "agent" else 1.0)
    return stack_status()


@app.post("/api/stack/stop")
def stack_stop() -> dict:
    if not _STACK_LOCK.acquire(blocking=False):
        return {**stack_status(), "busy": True,
                "detail": f"already {_STACK_BUSY['op'] or 'working'} — ignoring duplicate request"}
    _STACK_BUSY["op"] = "stopping"
    try:
        return _stack_stop_locked()
    finally:
        _STACK_BUSY["op"] = None
        _STACK_LOCK.release()


def _purge_run_logs() -> int:
    """Delete this run's logs when the operator powers the stack down.

    Service logs and PX4 .ulg flight logs regenerate on every start, and they were the
    single biggest disk consumer on this machine. Nothing else is touched: source, models,
    worlds, ~/.gazebo and the PX4 build are never deleted.
    """
    freed = 0
    try:
        for f in LOG_DIR.glob("*.log"):
            freed += f.stat().st_size
            f.unlink(missing_ok=True)
    except OSError:
        pass
    # PX4 per-instance flight logs (.ulg) — pure run artefacts
    try:
        for ulg in (PX4_DIR / "build" / "px4_sitl_default" / "rootfs").glob("*/log/**/*.ulg"):
            freed += ulg.stat().st_size
            ulg.unlink(missing_ok=True)
    except OSError:
        pass
    return freed


def _stack_stop_locked() -> dict:
    for name in reversed(_start_order()):
        SERVICES[name].stop()
    # `make px4_sitl` spawns gzserver/gzclient outside our process group, so
    # killing the group alone reliably leaves them behind.
    _reap_orphans()
    freed = _purge_run_logs()
    st = stack_status()
    st["logs_purged_mb"] = round(freed / 1e6, 1)
    return st


@app.get("/api/logs/size")
def logs_size() -> dict:
    """Report ROS log usage so the operator can decide before purging."""
    log_dir = Path.home() / ".ros" / "log"
    total = 0
    count = 0
    if log_dir.is_dir():
        for f in log_dir.rglob("*"):
            if f.is_file() and not f.is_symlink():
                try:
                    total += f.stat().st_size
                    count += 1
                except OSError:
                    pass
    return {"path": str(log_dir), "bytes": total,
            "human": f"{total / 1048576:.1f} MB", "files": count}


@app.post("/api/logs/purge")
def logs_purge(confirm: bool = False) -> dict:
    """Delete ROS logs. Requires ?confirm=true - destructive to history, so it
    never runs automatically and never as part of starting the simulation."""
    if not confirm:
        return {"ok": False, "detail": "refused: pass confirm=true",
                **logs_size()}
    log_dir = (Path.home() / ".ros" / "log").resolve()
    if _is_protected(log_dir) or log_dir == Path.home():
        return {"ok": False, "detail": "refused: protected path"}
    removed = 0
    for child in log_dir.glob("*"):
        if child.is_symlink():
            continue
        try:
            shutil.rmtree(child, ignore_errors=True) if child.is_dir() else child.unlink()
            removed += 1
        except OSError:
            continue
    return {"ok": True, "removed": removed}


@app.post("/api/stack/start/{name}")
def start_one(name: str) -> dict:
    if name in SERVICES:
        SERVICES[name].start()
    return stack_status()


@app.post("/api/stack/stop/{name}")
def stop_one(name: str) -> dict:
    if name in SERVICES:
        SERVICES[name].stop()
    return stack_status()


_BOILERPLATE_PHASES = ("takeoff", "land", "rtl", "return", "hold", "wait", "arm", "disarm")


def _mission_name(plan: dict) -> str:
    """A human name for this mission, taken from the planning agent's OWN phase names.

    Every swarm is driven by its own agent, so the console should show what that agent called
    the job instead of one fixed label. The first phase that is not boilerplate - takeoff,
    hold, return - is the one that actually describes the mission.
    """
    for ph in plan.get("phases", []) or []:
        name = (ph.get("name") or "").strip()
        if name and not name.lower().startswith(_BOILERPLATE_PHASES):
            return name.replace("_", " ").upper()
    return "MISSION"


def _mission_progress(mission_id: str) -> Tuple[str, int]:
    """How far the executor has got with `mission_id`, read from its own log.

    The coordinator only LOGS completion - it publishes no status topic - so the console had
    no way to tell a finished mission from a stalled one. Its log is therefore the source of
    truth: "plan <id>: N phase(s)" on intake, "plan <id> COMPLETE" when the last phase ends.
    Reading it here means the executor needs no change to report that it is done.
    """
    if not mission_id:
        return "NO MISSION", 0
    try:
        text = (LOG_DIR / "swarm.log").read_text(errors="ignore")[-200_000:]
    except OSError:
        return "NO MISSION", 0
    if f"plan {mission_id} COMPLETE" in text:
        return "COMPLETE", 0
    started = re.search(rf"plan {re.escape(mission_id)}: (\d+) phase", text)
    if started:
        return "EXECUTING", int(started.group(1))
    return "DISPATCHED", 0          # published, executor has not logged intake yet


@app.get("/api/mission/status")
def mission_status() -> dict:
    state, phases = _mission_progress(MISSION.mission_id)
    return {
        "llm_status": MISSION.llm_status,
        "waypoints": MISSION.waypoints,
        "running": MISSION.running,
        "log": MISSION.log[-60:],
        "mission_id": MISSION.mission_id,
        "mission_name": MISSION.mission_name,
        "mission_state": state,          # NO MISSION | DISPATCHED | EXECUTING | COMPLETE
        "mission_phases": phases,
    }


@app.post("/api/mission/run")
def mission_run(prompt: Optional[str] = None,
                req: Optional[MissionRequest] = None) -> dict:
    """Plan and dispatch a natural-language mission.

    Accepts the prompt as a query parameter (what the AEGIS console sends) or as a JSON
    body (the original aero_gcs client). Planning runs through ws_swarm's
    mission_interface: LLM -> validator -> MissionPlan on /mission_plan.
    """
    text = (prompt or (req.prompt if req else "") or "").strip()
    if not text:
        return {"ok": False, "detail": "empty prompt"}

    MISSION.log = []
    MISSION.llm_status = "LLM PLANNING MISSION…"
    result = plan_with_ws_swarm(text)
    if result.get("ok"):
        MISSION.llm_status = "LLM-VALIDATED MISSION PLAN DISPATCHED"
        MISSION.append(f"$ mission: {text}")
        MISSION.append(result.get("reply", ""))
    else:
        MISSION.llm_status = "PLAN REJECTED - see log"
        MISSION.append(f"$ mission: {text}")
        MISSION.append(f"rejected: {result.get('detail', '')}")
    return result


class TargetUpload(BaseModel):
    name: str = "target"
    dataURL: str = ""


@app.post("/api/mission/target")
def mission_target(req: TargetUpload) -> dict:
    """Register a target image. Classifies it so the agents know WHAT to look for — but does
    not arm anything: perception stays idle until a mission plan tasks a drone."""
    res = _classify_target(req.dataURL)
    if not res.get("ok"):
        return res
    entry = {"name": req.name, "label": res["label"], "conf": res["conf"], "t": time.time()}
    TARGETS.append(entry)
    return {
        "ok": True, "label": res["label"], "conf": res["conf"],
        "targets": [{k: t[k] for k in ("name", "label", "conf")} for t in TARGETS],
        "reply": (f'Target "{req.name}" identified as {res["label"]} '
                  f'({res["conf"] * 100:.0f}%). Agents remain passive until a mission tasks them.'),
    }


@app.post("/api/mission/target/clear")
def mission_target_clear() -> dict:
    TARGETS.clear()
    for d in _drone_ids():                      # return every detector to idle
        _ros_publish(f"/{d}/set_target", "std_msgs/msg/String", {"data": IDLE_CLASS})
    return {"ok": True, "targets": []}


@app.post("/api/sim/gps_denied")
def sim_gps_denied(enabled: bool = False) -> dict:
    """GPS-denied operation: the vehicles navigate on SLAM alone and the console shows the
    RTAB-Map. Recorded here so it survives a console reload."""
    was = bool(SIM_CONFIG.get("gps_denied"))
    SIM_CONFIG["gps_denied"] = bool(enabled)
    _apply_sim_config()          # rebuild the launch line (slam:=true/false)
    # rtabmap is launched WITH the stack, so a change only takes effect on the next power-on.
    needs_restart = (was != bool(enabled)) and bool(SERVICES["sitl"].running)
    return {"ok": True, "gps_denied": bool(enabled), "needs_restart": needs_restart,
            "detail": ("SLAM stack starts on the next power-on" if needs_restart else "")}


def _disengage_and_home(reason: str) -> Tuple[List[str], bool]:
    """Disengage every follower, then trigger the stack's own RTL. -> (disarmed, dispatched)

    Order is the whole point (see mission_abort). Both messages for every drone plus the
    abort trigger go out in ONE batch, so a 3-drone recall costs one DDS discovery round
    instead of seven - the difference between coming home in about a second and about eight.
    """
    ids = _drone_ids()
    messages = []
    for d in ids:
        messages.append({"topic": f"/{d}/set_target", "type": "std_msgs/msg/String",
                         "fields": {"data": IDLE_CLASS}})
        messages.append({"topic": f"/{d}/follow_reset", "type": "std_msgs/msg/String",
                         "fields": {"data": reason}})
    messages.append({"topic": "/mission/abort", "type": "std_msgs/msg/String",
                     "fields": {"data": reason}})

    results = _ros_publish_many(messages)
    disarmed = [d for d in ids if results.get(f"/{d}/set_target")]
    return disarmed, bool(results.get("/mission/abort"))


@app.post("/api/mission/rtl")
def mission_rtl() -> dict:
    """RETURN TO HOME. Same disengage-then-RTL path as abort, issued as a normal recall.

    A bare rtl cannot work on its own: target_follow holds PRIORITY_FOLLOW (25) and keeps
    refreshing it, so the controller rightly refuses a lower-priority rtl while a drone is
    pursuing. Releasing the followers first is what makes "come home" actually mean it.
    """
    disarmed, dispatched = _disengage_and_home("operator return to home")
    MISSION.llm_status = "RETURNING TO HOME" if dispatched else MISSION.llm_status
    return {
        "ok": dispatched,
        "disarmed": disarmed,
        "rtl_dispatched": dispatched,
        "reply": ("Returning to home." if dispatched
                  else "RTL not dispatched — safety_node is not running (is the stack up?)."),
    }


@app.post("/api/mission/abort")
def mission_abort() -> dict:
    """Disengage the target and bring everyone home.

    Order matters, and this is why a plain "return home" used to be ignored:

      target_follow publishes its pursuit at DroneTask PRIORITY_FOLLOW (25) and keeps
      refreshing it. A mission-issued rtl arrives at PRIORITY_NORMAL (10), so the
      controller correctly refuses it while the follow override is live — the drone just
      keeps following.

    So we disengage FIRST (detector back to the idle sentinel + follow_reset, which stops
    target_follow re-asserting), and only then trigger the abort. The abort itself uses the
    stack's OWN designed path: /mission/abort -> safety_node -> rtl at PRIORITY_SAFETY (255),
    which outranks FOLLOW. Nothing in the ROS nodes is modified; we just drive the existing
    contract in the right order.
    """
    disarmed, aborted = _disengage_and_home("operator abort — disengage and RTL")

    # Legacy: stop a mission subprocess if one is somehow still attached. Harmless no-op.
    p = MISSION.proc
    if p and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            pass
    MISSION.llm_status = "MISSION ABORTED BY OPERATOR"

    return {
        "ok": True,
        "disarmed": disarmed,
        "rtl_dispatched": aborted,
        "reply": (
            f"Disengaged {', '.join(disarmed) or 'no drones'}; "
            + ("returning to home at safety priority."
               if aborted else
               "RTL not dispatched — safety_node is not running (is the stack up?).")
        ),
    }


def _port_owner(port: int) -> Optional[int]:
    """PID currently listening on `port`, if any."""
    try:
        out = subprocess.run(["ss", "-ltnp"], capture_output=True,
                             text=True, timeout=5).stdout
    except (subprocess.SubprocessError, FileNotFoundError):
        return None
    for line in out.splitlines():
        if f":{port} " in line:
            m = re.search(r"pid=(\d+)", line)
            if m:
                return int(m.group(1))
    return None


def _free_port(port: int) -> bool:
    """Stop whatever owns `port`. Only used with --force."""
    pid = _port_owner(port)
    if pid is None:
        return True
    for sig in (signal.SIGINT, signal.SIGKILL):
        try:
            os.kill(pid, sig)
        except (ProcessLookupError, PermissionError):
            return _port_owner(port) is None
        time.sleep(2)
        if _port_owner(port) is None:
            return True
    return _port_owner(port) is None


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="AEROMAST control API")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--force", action="store_true",
                    help="stop whatever already holds the port, then start")
    cli = ap.parse_args()

    owner = _port_owner(cli.port)
    if owner is not None:
        if cli.force:
            print(f"[INFO] port {cli.port} held by pid {owner} — stopping it")
            if not _free_port(cli.port):
                print(f"[ERROR] could not free port {cli.port} (pid {owner})")
                raise SystemExit(1)
        else:
            print(
                f"[ERROR] port {cli.port} is already in use by pid {owner}.\n"
                f"        An older control_api is probably still running.\n"
                f"        Restart it with:   python3 control_api.py --force\n"
                f"        Or stop it with:   kill {owner}"
            )
            raise SystemExit(1)

    print(f"[INFO] AEGIS control API :{cli.port}  (root: {PIPELINE_ROOT})")
    uvicorn.run(app, host="0.0.0.0", port=cli.port, log_level="warning")
