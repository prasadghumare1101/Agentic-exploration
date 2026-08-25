#!/usr/bin/env bash
# Start the AEGIS OPS CONSOLE — one command, one URL.
#
#   ~/aegis_gcs/start.sh          # console + control API
#   ~/aegis_gcs/start.sh --dev    # same, but the UI runs on the Vite dev server (HMR)
#
# The control API supervises everything else (uXRCE agent, PX4 SITL/Gazebo, rosbridge,
# telemetry, video, swarm stack) — those start when you press POWER in the console.
#
# NOTE: deliberately no `set -u`; ROS 2's setup.bash references unset vars.
set -o pipefail

GCS=~/ws_swarm/aegis_gcs
# AEGIS owns its backend. drone_llm_pipeline/aero_gcs was studied as a REFERENCE only —
# nothing here reads from or writes to that project.
BACKEND="$GCS/backend"
UI_PORT=5200
API_PORT=8000

source /opt/ros/humble/setup.bash 2>/dev/null
source ~/ws_swarm/install/setup.bash 2>/dev/null
export FASTRTPS_DEFAULT_PROFILES_FILE=~/ws_swarm/config/udp_only.xml
export DISPLAY="${DISPLAY:-:0}"

# Always start clean: a stale API holding :8000 is the usual "nothing responds" cause.
for p in $(pgrep -f "control_api.py"); do kill -9 "$p" 2>/dev/null; done
for p in $(pgrep -f "vite.*--port $UI_PORT"); do kill -9 "$p" 2>/dev/null; done
sleep 1

# Control API (FastAPI :8000) — supervises the simulation stack + LLM missions.
( cd "$BACKEND" && exec python3 control_api.py ) > /tmp/aegis_control_api.log 2>&1 &
API_PID=$!

# UI: dev server for hacking, otherwise a static preview of the production build.
cd "$GCS"
if [ "${1:-}" = "--dev" ]; then
  UI_CMD="npx vite --host --port $UI_PORT"
else
  npm run build > /tmp/aegis_ui_build.log 2>&1 \
    && echo "  dashboard built" \
    || echo "  build failed (see /tmp/aegis_ui_build.log) — serving previous build"
  UI_CMD="npx vite preview --host --port $UI_PORT"
fi

echo "──────────────────────────────────────────────────────────"
echo "  AEGIS OPS CONSOLE   →  http://localhost:$UI_PORT/"
echo "  control API         →  http://localhost:$API_PORT/  (pid $API_PID)"
echo "  logs                →  /tmp/aegis_control_api.log"
echo ""
echo "  In the console: choose SINGLE/SWARM + world + moving target,"
echo "  then press POWER to launch Gazebo and the full stack."
echo "──────────────────────────────────────────────────────────"

# Stop the API when the UI server exits, so one Ctrl-C tears the whole console down.
trap 'kill -9 $API_PID 2>/dev/null' EXIT INT TERM
exec $UI_CMD
