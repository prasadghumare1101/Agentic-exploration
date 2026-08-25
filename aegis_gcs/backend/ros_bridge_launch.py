#!/usr/bin/env python3
"""Standalone launcher for the ROS 2 websocket bridge (port 9090).

The dashboard's useROS hook talks to rosbridge over ws://localhost:9090.
control_api.py normally starts this for you; this script exists so the bridge
can also be run on its own for debugging.

Requires:  sudo apt install ros-humble-rosbridge-suite
Run:       python3 ros_bridge_launch.py
"""
from __future__ import annotations

import os
import subprocess
import sys

ROS_SETUP = os.environ.get("ROS_SETUP", "/opt/ros/humble/setup.bash")


def launch_bridge() -> int:
    print("[INFO] Starting ROS 2 WebSocket Bridge on port 9090...")
    cmd = (f"source {ROS_SETUP} && "
           "exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml")
    try:
        return subprocess.call(["bash", "-c", cmd])
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(launch_bridge())
