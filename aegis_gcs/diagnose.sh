#!/usr/bin/env bash
# Why isn't the drone taking off? Run this WHILE the sim is powered on.
#
#   ~/ws_swarm/aegis_gcs/diagnose.sh
#
# It walks the execution chain in order and stops being polite about whichever link is
# broken:
#   LLM -> /mission_plan -> coordinator_node -> /px4_N/task -> drone_controller -> PX4 -> flight
set -o pipefail

source /opt/ros/humble/setup.bash 2>/dev/null
source ~/ws_swarm/install/setup.bash 2>/dev/null
export FASTRTPS_DEFAULT_PROFILES_FILE=~/ws_swarm/config/udp_only.xml
LOGS=~/ws_swarm/aegis_gcs/backend/logs
D="${1:-px4_1}"

ok()   { echo "  [ OK ] $*"; }
bad()  { echo "  [FAIL] $*"; }
info() { echo "         $*"; }

echo "─── 1. simulation processes ───"
pgrep -x gzserver >/dev/null && ok "gazebo (gzserver) running" || bad "gazebo NOT running — press POWER in the console"
pgrep -x px4 >/dev/null && ok "PX4 SITL running ($(pgrep -x px4 | wc -l) instance(s))" || bad "PX4 NOT running"
pgrep -f "MicroXRCEAgent udp4" >/dev/null && ok "uXRCE-DDS agent running" || bad "DDS agent NOT running"

echo "─── 2. ws_swarm executor stack ───"
pgrep -f coordinator_node >/dev/null && ok "coordinator_node (turns the plan into tasks)" \
  || bad "coordinator_node NOT running — no plan can execute"
pgrep -f drone_controller >/dev/null && ok "drone_controller ($(pgrep -f drone_controller | wc -l))" \
  || bad "drone_controller NOT running — nothing drives PX4"
pgrep -f safety_node >/dev/null && ok "safety_node" || info "safety_node not running"

echo "─── 3. did the plan arrive and parse? ───"
if [ -f "$LOGS/swarm.log" ]; then
  if grep -q "bad plan_json" "$LOGS/swarm.log"; then
    bad "coordinator rejected the plan JSON:"; grep "bad plan_json" "$LOGS/swarm.log" | tail -2 | sed 's/^/         /'
  fi
  # Encoding-safe: the coordinator logs "plan <id>: N phase(s) - starting" with an em dash,
  # which a literal grep can miss. Match on the stable part instead.
  P=$(grep -ac "phase(s)" "$LOGS/swarm.log" 2>/dev/null)
  DISP=$(grep -ac "task ->" "$LOGS/swarm.log" 2>/dev/null)
  if [ "${P:-0}" -gt 0 ]; then ok "coordinator accepted $P plan(s)"
  elif [ "${DISP:-0}" -gt 0 ]; then ok "tasks were dispatched ($DISP) - plan ran"
  else bad "no plan accepted yet (issue a mission in the console)"; fi
  grep -E "ignoring .* safety override" "$LOGS/swarm.log" | tail -3 | sed 's/^/         /'
  grep -E "task -> " "$LOGS/swarm.log" | tail -3 | sed 's/^/         /'
else
  bad "no $LOGS/swarm.log — the ws_swarm stack never started"
fi

echo "─── 4. is a task reaching the drone? ───"
T=$(timeout 6 ros2 topic echo --once "/$D/task" swarm_msgs/msg/DroneTask 2>/dev/null | grep -E "^action:|priority:" | tr '\n' ' ')
[ -n "$T" ] && ok "/$D/task -> $T" || bad "/$D/task is silent — the coordinator is not dispatching to $D"

echo "─── 5. is PX4 being driven, and can it arm? ───"
S=$(timeout 6 ros2 topic hz "/$D/fmu/in/trajectory_setpoint" 2>/dev/null | grep -m1 "average rate")
[ -n "$S" ] && ok "offboard setpoints flowing ($S)" || bad "no setpoints — drone_controller is not publishing"
L=$(timeout 6 ros2 topic echo --once --qos-reliability best_effort \
      "/$D/fmu/out/vehicle_local_position_v1" px4_msgs/msg/VehicleLocalPosition 2>/dev/null)
if [ -n "$L" ]; then
  ALT=$(echo "$L" | grep -m1 "^z:" | awk '{printf "%.2f", -$2}')
  XY=$(echo "$L" | grep -m1 "xy_valid:" | awk '{print $2}')
  HDG=$(echo "$L" | grep -m1 "heading_good_for_control:" | awk '{print $2}')
  ok "altitude ${ALT} m | xy_valid=$XY | heading_good_for_control=$HDG"
  [ "$HDG" = "false" ] && info "heading not ready -> PX4 refuses to arm. The world needs <magnetic_field> in <physics>."
  [ "${ALT%.*}" -ge 1 ] 2>/dev/null && ok "AIRBORNE" || info "still on the ground"
else
  bad "no vehicle_local_position — PX4<->DDS link is down"
fi

echo "─── summary ───"
echo "  Full logs: $LOGS/{swarm,sitl,agent,telemetry,video}.log"
echo "  Live executor:  tail -f $LOGS/swarm.log | grep -E 'plan|task ->|ignoring|arm'"
