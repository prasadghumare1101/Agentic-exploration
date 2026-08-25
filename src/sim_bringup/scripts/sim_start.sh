#!/usr/bin/env bash
# Reference startup order for the Stage 1 simulation stack (Gazebo Classic).
# Documentation-as-script: run the pieces in separate terminals so you can watch
# each one. It intentionally does NOT kill or manage existing processes.
#
# Prereqs already working on this machine: PX4-Autopilot SITL (gazebo-classic),
# MicroXRCEAgent, Gazebo Classic, and the custom `iris` model. Nothing here
# touches those assets.
#
# NAMESPACE FACT (from PX4 ROMFS/.../rcS): instance 0 gets NO namespace (bare
# /fmu/...), instances >0 get "-n px4_<instance>". The gazebo-classic
# sitl_multiple_run.sh spawns instances starting at 1, so -n K yields exactly
# /px4_1 .. /px4_K. A plain `make px4_sitl gazebo-classic` is instance 0 =
# UNNAMESPACED — avoid it for the swarm.
set -e

echo "Stage 1 sim startup order (run each in its own terminal):"
echo
echo "0) One-time build (already done via DONT_RUN=1 make px4_sitl gazebo-classic):"
echo "     cd ~/PX4-Autopilot && DONT_RUN=1 make px4_sitl gazebo-classic"
echo
echo "1) Micro XRCE-DDS agent (SIM, udp4) — start this first:"
echo "     MicroXRCEAgent udp4 -p 8888"
echo "   NOTE: the serial agent (serial --dev /dev/ttyUSB0) is real hardware — leave it alone."
echo
echo "2) PX4 SITL + Gazebo Classic."
echo "   One-drone proof (single iris as instance 1 -> /px4_1):"
echo "     cd ~/PX4-Autopilot"
echo "     ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m iris -n 1 -w empty"
echo "   Full swarm (instances 1..3 -> /px4_1 /px4_2 /px4_3):"
echo "     ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m iris -n 3 -w empty"
echo
echo "3) Confirm the namespaces the sim actually created:"
echo "     source ~/ws_swarm/install/setup.bash"
echo "     ros2 topic list | grep fmu    # expect /px4_1/fmu/... (and /px4_2, /px4_3 for the swarm)"
echo
echo "4) ROS 2 Stage 1 layer (per drone; swarm singletons start with px4_1):"
echo "     ros2 launch sim_bringup stage1_single.launch.py drone_id:=px4_1"
echo
echo "5) Send a mission:"
echo "     ros2 run mission_examples mission_sender --ros-args \\"
echo "       -p file:=\$(ros2 pkg prefix mission_examples)/share/mission_examples/missions/takeoff_hold.json"
