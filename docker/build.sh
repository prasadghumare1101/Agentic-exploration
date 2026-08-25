#!/usr/bin/env bash
# Build the AEGIS swarm image.
#
# Docker can only copy from inside the build context, and the pieces that make this rig
# specific live outside the workspace: the hand-modified iris model, the worlds, and the
# local Gazebo model cache. This script stages them into docker/context/ first, so the image
# is built from THIS workstation's assets rather than whatever upstream currently ships.
#
#   ./docker/build.sh              build image aegis-swarm:latest
#   ./docker/build.sh mytag        build image aegis-swarm:mytag
set -euo pipefail

TAG="${1:-latest}"
IMAGE="aegis-swarm:${TAG}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${HERE}/.." && pwd)"
PX4="${PX4_DIR:-${HOME}/PX4-Autopilot}"
GZ_MODELS="${HOME}/.gazebo/models"
CTX="${HERE}/context"

say() { echo "  $*"; }
die() { echo "ERROR: $*" >&2; exit 1; }

echo "── staging build context ──────────────────────────────────────"

# The heavy world models are not in the repository (baylands alone is 393 MB). Use the local
# cache when this workstation has one, otherwise fall back to whatever the repo ships and warn
# - a missing model renders an EMPTY world, which is easy to mistake for a broken build.
if [[ ! -d "${GZ_MODELS}" ]]; then
    echo "  NOTE: no ${GZ_MODELS} — run ./scripts/fetch_gazebo_models.sh first for full worlds."
    GZ_MODELS="${WS}/assets/gazebo_models"
fi
[[ -d "${GZ_MODELS}" ]] || die "no Gazebo models found; run ./scripts/fetch_gazebo_models.sh"

# The model and worlds ship IN the repository (ws_swarm/assets), so a clone on any machine
# builds the same drone. The PX4 checkout is only a fallback for a workstation whose assets/
# has not been populated yet - a fresh clone must never silently fall back to the sensorless
# upstream iris.
if [[ -f "${WS}/assets/models/iris/iris.sdf.jinja" ]]; then
    SRC_IRIS="${WS}/assets/models/iris"
    SRC_WORLDS="${WS}/assets/worlds"
    say "assets source   repository (ws_swarm/assets)"
else
    SRC_IRIS="${PX4}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris"
    SRC_WORLDS="${PX4}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds"
    say "assets source   PX4 checkout at ${PX4} (repo assets/ is empty)"
fi
[[ -f "${SRC_IRIS}/iris.sdf.jinja" ]] || die "iris.sdf.jinja missing at ${SRC_IRIS}"

rm -rf "${CTX}"
mkdir -p "${CTX}"/{models,worlds,gazebo_models,ws_swarm}

# --- the drone itself -------------------------------------------------------
cp -r "${SRC_IRIS}" "${CTX}/models/iris"
rm -f "${CTX}/models/iris/"*.bak-*        # local editing backups, not part of the model
say "iris model      $(du -sh "${CTX}/models/iris" | cut -f1)  ($(ls "${CTX}/models/iris" | tr '\n' ' '))"
say "  sensors:      $(grep -c "<sensor name=" "${CTX}/models/iris/iris.sdf.jinja") declared"
say "  fingerprint:  $(md5sum "${CTX}/models/iris/iris.sdf.jinja" | cut -c1-32)"

# --- worlds (includes the project's own, e.g. clear_follow) -----------------
cp "${SRC_WORLDS}"/*.world "${CTX}/worlds/"
say "worlds          $(ls "${CTX}/worlds" | wc -l) files"

# --- local model cache: every model a world references must be here ---------
cp -r "${GZ_MODELS}/." "${CTX}/gazebo_models/"
say "gazebo models   $(ls "${CTX}/gazebo_models" | wc -l) models, $(du -sh "${CTX}/gazebo_models" | cut -f1)"

# --- the workspace: sources only, never build/ install/ log/ node_modules ---
cp -r "${WS}/src"    "${CTX}/ws_swarm/src"
find "${CTX}/ws_swarm/src" -name ".env" -delete      # secrets are passed at run time, never baked in

# px4_msgs and px4-offboard are recorded in this repo as gitlinks (mode 160000) but there is
# no .gitmodules, so a plain `git clone` leaves both directories EMPTY. colcon then builds 10
# packages instead of 12 and px4_msgs is simply absent from the image, which kills every node
# that talks to PX4 - telemetry_node dies with `ModuleNotFoundError: No module named
# 'px4_msgs'` and the console shows a running sim with no data at all.
#
# Fetch them at the exact commits this project was built against whenever the working copy is
# empty. A populated checkout (this workstation) is used as-is and never re-fetched, so the
# image still tracks local edits.
fetch_pinned() {                     # name  url  sha
    local dst="${CTX}/ws_swarm/src/$1"
    if [[ -f "${dst}/package.xml" ]] || compgen -G "${dst}/*/package.xml" >/dev/null; then
        say "$1"$'\t'"local checkout ($(ls -A "${dst}" | wc -l) entries)"
        return 0
    fi
    say "$1"$'\t'"empty gitlink -> fetching ${3:0:12}"
    rm -rf "${dst}"
    git clone --quiet "$2" "${dst}" >/dev/null 2>&1 || die "cannot clone $2 (network?)"
    git -C "${dst}" checkout --quiet "$3" 2>/dev/null || die "commit $3 missing from $2"
    rm -rf "${dst}/.git"
}
fetch_pinned px4_msgs     https://github.com/PX4/px4_msgs.git \
                          52f10548124b94b64f19860388c06179f8d29fcf
fetch_pinned px4-offboard https://github.com/Jaeyoung-Lim/px4-offboard.git \
                          3a765eff66c1c7b723f9be1d98bfa72c3c885e03

# A missing px4_msgs is the single most damaging thing that can reach the image, and it fails
# silently at runtime rather than at build time. Refuse to build without it.
[[ -f "${CTX}/ws_swarm/src/px4_msgs/package.xml" ]] \
    || die "px4_msgs has no package.xml - the image would build but telemetry and every PX4 node would die at runtime"
cp -r "${WS}/config" "${CTX}/ws_swarm/config"
cp "${WS}/build.sh" "${CTX}/ws_swarm/build.sh"     # swarm_msgs needs its two-step build
cp -r "${WS}/models" "${CTX}/ws_swarm/models"
mkdir -p "${CTX}/ws_swarm/aegis_gcs"
# start.sh is what the container's entrypoint execs, and diagnose.sh is the field tool for a
# drone that will not fly. Leaving them out builds a perfectly good image that dies on the
# first run with "No such file or directory".
for item in backend src public index.html package.json package-lock.json \
            vite.config.js tailwind.config.js postcss.config.js \
            start.sh diagnose.sh; do
    [[ -e "${WS}/aegis_gcs/${item}" ]] && cp -r "${WS}/aegis_gcs/${item}" "${CTX}/ws_swarm/aegis_gcs/"
done
rm -rf "${CTX}/ws_swarm/aegis_gcs/backend/logs"
chmod +x "${CTX}/ws_swarm/aegis_gcs/start.sh" "${CTX}/ws_swarm/aegis_gcs/diagnose.sh" 2>/dev/null || true
say "ws_swarm        $(du -sh "${CTX}/ws_swarm" | cut -f1)"
say "detector weights $(ls "${CTX}/ws_swarm/models"/*.pt 2>/dev/null | xargs -n1 basename | tr '\n' ' ')"

cp "${HERE}/entrypoint.sh"          "${CTX}/entrypoint.sh"
cp "${HERE}/constraints.txt"       "${CTX}/constraints.txt"
cp "${HERE}/px4_namespace_patch.py" "${CTX}/px4_namespace_patch.py"

echo
echo "── verifying the staged model is THIS rig's, not upstream ─────"
if diff -q "${SRC_IRIS}/iris.sdf.jinja" "${CTX}/models/iris/iris.sdf.jinja" >/dev/null; then
    say "OK: staged iris matches the workstation byte for byte"
else
    die "staged iris differs from the workstation copy"
fi

echo
echo "── building ${IMAGE} ──────────────────────────────────────────"
say "context: $(du -sh "${CTX}" | cut -f1)"
docker build -f "${HERE}/Dockerfile" -t "${IMAGE}" "${HERE}"

echo
say "built ${IMAGE}  ($(docker image inspect "${IMAGE}" --format '{{.Size}}' | awk '{printf "%.1f GB", $1/1e9}'))"
say "run it with: ${HERE}/run.sh ${TAG}"
