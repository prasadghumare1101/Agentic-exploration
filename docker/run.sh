#!/usr/bin/env bash
# Run the AEGIS swarm container.
#
#   ./docker/run.sh            start the console (default)
#   ./docker/run.sh latest shell   drop into a shell inside the image
#
# The flags below are not optional extras — each one prevents a specific failure:
#
#   --gpus all              Gazebo renders and YOLO infers on the GPU. Without it gzclient
#                           falls back to software GL and the sim crawls.
#   -e DISPLAY + X11 mount  Gazebo Classic's GUI is an X client. Without these it cannot open
#                           a window and you get a headless sim with no view.
#   --network host          DDS discovery is multicast. Under Docker's default bridge the
#                           container's nodes and any host-side tools never find each other.
#   ROS_DOMAIN_ID           Isolates this container from a ROS stack running on the host, so
#                           two coordinators can never fight over the same /px4_N/task topic.
#   --ipc=host + shm-size   Gazebo and rendering need real shared memory; the 64 MB default
#                           is far too small and shows up as texture/plugin failures.
set -euo pipefail

TAG="${1:-latest}"
MODE="${2:-console}"
IMAGE="aegis-swarm:${TAG}"
NAME="aegis-swarm"
DOMAIN="${ROS_DOMAIN_ID:-42}"

docker image inspect "${IMAGE}" >/dev/null 2>&1 \
    || { echo "ERROR: ${IMAGE} not found — run ./docker/build.sh first" >&2; exit 1; }

# Let the container's X clients talk to this display. Scoped to the local user, not xhost +.
if [[ -n "${DISPLAY:-}" ]]; then
    xhost +SI:localuser:root >/dev/null 2>&1 || \
        echo "  note: could not run xhost; the Gazebo window may be refused by X"
fi

# Secrets: --env-file keeps the token out of `docker inspect` and out of shell history,
# which -e HF_TOKEN=... does not. Falls back to mounting an existing .env read-only.
SECRET_FLAGS=()
ENV_FILE="${AEGIS_ENV_FILE:-${PWD}/.env}"
HOST_ENV="${PWD}/src/mission_interface/.env"
if [[ -f "${ENV_FILE}" ]]; then
    SECRET_FLAGS=(--env-file "${ENV_FILE}")
    echo "  LLM credentials: --env-file ${ENV_FILE}"
elif [[ -f "${HOST_ENV}" ]]; then
    SECRET_FLAGS=(-v "${HOST_ENV}:/root/ws_swarm/src/mission_interface/.env:ro")
    echo "  LLM credentials: mounting ${HOST_ENV} read-only"
elif [[ -n "${HF_TOKEN:-}" ]]; then
    SECRET_FLAGS=(-e "HF_TOKEN=${HF_TOKEN}")
    echo "  LLM credentials: HF_TOKEN from the environment"
else
    echo "  NOTE: no HF_TOKEN found — natural-language missions will be unavailable."
    echo "        Create ./.env with HF_TOKEN=... or export HF_TOKEN before running."
fi

# GPU access. --runtime=nvidia is tried FIRST because it is the only form that works on every
# toolkit configuration. Newer nvidia-container-toolkit installs default to CDI mode, where
# --gpus all invokes the prestart hook directly and the runtime refuses outright:
#
#   invoking the NVIDIA Container Runtime Hook directly (e.g. specifying the docker --gpus
#   flag) is not supported. Please use the NVIDIA Container Runtime instead
#
# The image already declares NVIDIA_VISIBLE_DEVICES/NVIDIA_DRIVER_CAPABILITIES, which is what
# --runtime=nvidia reads to decide which devices and which capabilities (incl. GL for gzclient)
# to expose, so no extra flag is needed alongside it.
GPU_FLAGS=()
if docker info --format '{{range $r, $_ := .Runtimes}}{{$r}} {{end}}' 2>/dev/null | grep -qw nvidia; then
    GPU_FLAGS=(--runtime=nvidia)
elif docker info 2>/dev/null | grep -qi nvidia; then
    GPU_FLAGS=(--gpus all)
else
    echo "  WARNING: no NVIDIA container runtime detected — running WITHOUT GPU."
    echo "           Gazebo will use software rendering and YOLO will run on CPU."
fi

docker run --rm -it \
    --name "${NAME}" \
    "${GPU_FLAGS[@]}" \
    "${SECRET_FLAGS[@]}" \
    --network host \
    --ipc=host \
    --shm-size=2g \
    -e DISPLAY="${DISPLAY:-}" \
    -e QT_X11_NO_MITSHM=1 \
    -e ROS_DOMAIN_ID="${DOMAIN}" \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev/dri:/dev/dri \
    "${IMAGE}" "${MODE}"
