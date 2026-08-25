#!/usr/bin/env bash
# Container entrypoint. Sources the ROS environment, then runs the requested mode.
#
#   console   start the AEGIS backend + console        (default)
#   shell     interactive bash with everything sourced
#   <cmd...>  run an arbitrary command in the sourced environment
#
# No `set -u` while sourcing: ROS 2's setup.bash reads $AMENT_TRACE_SETUP_FILES with no
# default, so under -u the entrypoint dies on line 8 of it before the console ever starts.
# aegis_gcs/start.sh carries the same note for the same reason. -u is switched on straight
# after, so it still guards this script's own code.
set -e -o pipefail

source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"

set -u

# The workspace ships a UDP-only FastDDS profile. Shared memory transport exhausts its port
# pool with several vehicles and drones then become undiscoverable to each other.
export FASTRTPS_DEFAULT_PROFILES_FILE="${WS}/config/udp_only.xml"

# --------------------------------------------------------------------------- #
# LLM credentials.
#
# The token is NEVER baked into the image: an ENV or ARG in a Dockerfile is written into the
# layer history and can be read back by anyone who pulls it. It arrives at RUN time instead
# (--env-file is preferred over -e, since -e is visible in `docker inspect` and shell history)
# and is written to the .env file llm_client._find_env_file() insists on, inside the
# container's writable layer, which disappears with the container.
# --------------------------------------------------------------------------- #
write_env_file() {
    local env_file="${WS}/src/mission_interface/.env"
    [[ -f "${env_file}" ]] && return 0            # a mounted .env wins; never overwrite it
    if [[ -z "${HF_TOKEN:-}" ]]; then
        echo "NOTE: HF_TOKEN not set — flight, detection and follow all work, but natural-" >&2
        echo "      language missions will fail. Pass it with:  --env-file your.env" >&2
        return 0
    fi
    mkdir -p "$(dirname "${env_file}")"
    umask 077
    {
        echo "HF_TOKEN=${HF_TOKEN}"
        # These defaults must name models the HF router actually serves. Qwen2.5-32B-Instruct
        # and Meta-Llama-3.1-8B-Instruct are NOT on it - the first is not served at all, the
        # second is published as Llama-3.1-8B-Instruct (no "Meta-" prefix). Both were rejected
        # with "not supported by any provider you have enabled", which reads like an account
        # permissions problem but is really a wrong model id.
        # Check the current list with:
        #   curl -H "Authorization: Bearer $HF_TOKEN" https://router.huggingface.co/v1/models
        echo "HF_MODEL_ID_PRIMARY=${HF_MODEL_ID_PRIMARY:-Qwen/Qwen2.5-72B-Instruct}"
        echo "HF_MODEL_ID_SECONDARY=${HF_MODEL_ID_SECONDARY:-meta-llama/Llama-3.1-8B-Instruct}"
    } > "${env_file}"
    echo "  LLM credentials written to ${env_file} (mode 600, container-local)"
}

banner() {
    echo "──────────────────────────────────────────────────────────────"
    echo " AEGIS swarm container"
    echo "   ROS 2 $(printenv ROS_DISTRO)   domain ${ROS_DOMAIN_ID}   RMW ${RMW_IMPLEMENTATION}"
    echo "   PX4       $(cat /etc/px4-version 2>/dev/null || echo '?')"
    echo "   drone     $(ls "${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/" | tr '\n' ' ')"
    echo "   display   ${DISPLAY:-<none — GUI disabled>}"
    echo "   console   http://localhost:5200      API http://localhost:8000"
    echo "──────────────────────────────────────────────────────────────"
}

case "${1:-console}" in
    console)
        banner
        write_env_file
        # Fail loudly here rather than leaving the operator with a black Gazebo window.
        if [[ -z "${DISPLAY:-}" ]]; then
            echo "WARNING: DISPLAY is unset — Gazebo's GUI cannot open." >&2
            echo "         Run with -e DISPLAY=\$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix" >&2
        fi
        # Hand over to the project's OWN launcher, so a container behaves exactly like the
        # workstation: same start order, same ports, same cleanup. Duplicating its logic here
        # would be a second thing to keep in sync, and the two would drift.
        exec "${WS}/aegis_gcs/start.sh"
        ;;
    shell)
        banner
        write_env_file
        exec bash
        ;;
    *)
        exec "$@"
        ;;
esac
