#!/bin/bash
# Reliable workspace build. The swarm_msgs interface package fails under
# `--symlink-install` (ament tries to symlink over an existing directory), so it is
# built plain and everything else symlinked. Run this instead of a bare `colcon build`.
set -e
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/*/setup.bash
colcon build --packages-select swarm_msgs
colcon build --symlink-install --packages-skip swarm_msgs
echo "build.sh: all packages built."
