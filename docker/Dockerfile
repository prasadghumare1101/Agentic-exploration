# AEGIS swarm — full simulation stack in one image.
#
# Pinned to the exact versions this project was developed against, so a container on another
# machine behaves the same as this workstation:
#
#   Ubuntu 22.04.5   ROS 2 Humble   Gazebo Classic 11.10.2
#   PX4-Autopilot    v1.17.0-alpha1-29-gd2548ced9d   (commit d2548ced9d)
#   Micro-XRCE-DDS-Agent  v2.4.3
#   torch 2.11.0+cu130   ultralytics 8.3.223   opencv 4.9.0.80   numpy 1.26.4
#
# The DRONE MODEL is not taken from upstream PX4. build.sh stages this workstation's
# hand-modified iris (sensors, cameras, lidar) and its worlds into the build context, and
# they are copied in AFTER the PX4 build so nothing upstream can overwrite them.
#
# Build:  ./docker/build.sh          Run:  ./docker/run.sh
FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive

# Gazebo needs the GL/GLX capabilities, not just compute, or gzclient renders nothing.
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

# --------------------------------------------------------------------------- #
# System packages
# --------------------------------------------------------------------------- #
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git ninja-build ccache \
        python3-pip python3-dev python3-venv \
        gazebo libgazebo-dev \
        ros-humble-rosbridge-server \
        ros-humble-rtabmap-ros ros-humble-rtabmap-slam \
        ros-humble-rqt-image-view ros-humble-tf2-ros ros-humble-vision-msgs \
        ros-humble-cv-bridge ros-humble-image-transport ros-humble-compressed-image-transport \
        ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins \
        curl wget gnupg lsb-release ca-certificates \
        libeigen3-dev libopencv-dev protobuf-compiler libprotobuf-dev \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-base \
        x11-apps mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Node 20 for the AEGIS console build
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------------------------------- #
# Python — versions pinned to the workstation so inference results match exactly.
#
# PIP_CONSTRAINT applies constraints.txt to EVERY pip invocation in this image, including
# PX4's own setup script. Without it, an unconstrained transitive dependency (numpy>=1.13)
# resolves to whatever is newest on build day, and the image silently stops matching the
# system it was validated on.
# --------------------------------------------------------------------------- #
COPY context/constraints.txt /etc/pip-constraints.txt
ENV PIP_CONSTRAINT=/etc/pip-constraints.txt
# torch/torchvision ONLY from the PyTorch index. --index-url REPLACES PyPI for the whole
# command, so bundling anything else here makes pip look for it on an index that does not
# carry it ("No matching distribution found for opencv-python"). Everything else installs
# from PyPI in a separate step.
RUN pip3 install --no-cache-dir --index-url https://download.pytorch.org/whl/cu130 \
        torch==2.11.0 torchvision==0.26.0

# ultralytics pulls its own opencv/numpy if they are absent, so pin ours FIRST and let it
# accept them, otherwise the versions drift from the ones this project was validated against.
# opencv-contrib-python carries the legacy/contrib trackers. detector.py seeds a CSRT tracker
# from each YOLO hit and follows those pixels every frame - that is what turns intermittent
# detections into a continuous lock for target_follow. Plain opencv-python has NEITHER
# cv2.TrackerCSRT_create NOR cv2.legacy, so both branches of detector.py's tracker factory
# raise and every cycle dies with:
#     inference failed: module 'cv2' has no attribute 'legacy'
# Detection still logs hits, so the failure hides in plain sight while follow never locks on.
# The workstation has both wheels at the same version; matching that exactly is what makes
# the container behave like it.
RUN pip3 install --no-cache-dir \
        numpy==1.26.4 \
        opencv-python==4.9.0.80 \
        opencv-contrib-python==4.9.0.80 \
 && pip3 install --no-cache-dir \
        ultralytics==8.3.223 \
        fastapi==0.139.2 uvicorn==0.41.0 pydantic==2.12.5 \
        psutil==5.9.0 pyulog==1.2.2 \
        empy==3.3.4 pyros-genmsg==0.5.8 jinja2==3.1.6 toml==0.10.2 lark==1.1.1 \
        huggingface-hub==1.4.1 python-dotenv==1.2.1

# --------------------------------------------------------------------------- #
# Micro-XRCE-DDS-Agent v2.4.3 — the PX4 <-> ROS 2 bridge
# --------------------------------------------------------------------------- #
# This project uses a SUPERBUILD: the first configure produces a Makefile whose only target
# is 'all', which fetches and builds Fast-DDS and friends and only then regenerates the real
# project - the one that has an 'install' target. Asking for 'install' up front fails with
# "No rule to make target 'install'". So: build 'all' first, then install.
RUN git clone -b v2.4.3 --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /opt/xrce \
 && cmake -S /opt/xrce -B /opt/xrce/build -DCMAKE_BUILD_TYPE=Release \
 && cmake --build /opt/xrce/build -j"$(nproc)" \
 && cmake --build /opt/xrce/build --target install \
 && ldconfig && rm -rf /opt/xrce/build

# --------------------------------------------------------------------------- #
# PX4-Autopilot at the exact commit, built for SITL
# --------------------------------------------------------------------------- #
ARG PX4_COMMIT=d2548ced9d5b2b14da3cdfefb6d564b3a189720f
ENV PX4_DIR=/opt/PX4-Autopilot
RUN git init ${PX4_DIR} \
 && cd ${PX4_DIR} \
 && git remote add origin https://github.com/PX4/PX4-Autopilot.git \
 && git fetch --depth 1 origin ${PX4_COMMIT} \
 && git checkout FETCH_HEAD \
 && git submodule update --init --recursive --depth 1 \
 && bash Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools \
 && rm -rf /var/lib/apt/lists/*

# Build the firmware. Kept as its own step so a failed compile FAILS THE IMAGE BUILD.
#
# This previously ran the compile and the cleanup in one '&&' chain ending in '|| true'. Shell
# gives && and || equal precedence and evaluates left to right, so that trailing '|| true'
# neutralised the compile's exit status too: a CMake error produced a 12.8 GB image with no
# px4 binary and no Gazebo plugins, and the build still reported success. Nothing downstream
# would notice until the sim refused to start.
RUN cd ${PX4_DIR} && DONT_RUN=1 make px4_sitl gazebo-classic

# Assert the artefacts exist, record the version, then shed what the runtime does not need:
#   - /etc/px4-version is written BEFORE the git metadata goes, so the container banner can
#     still report exactly which PX4 it runs;
#   - .git plus every submodule's objects under .git/modules is well over a gigabyte for a
#     tree that is never updated again.
# 'set -e' covers the assertions; 'set +e' covers the best-effort cleanup, so a stray rm can
# never mask a missing binary the way the old chain did.
# NOTE: no '#' comments inside this RUN - Docker joins continued lines before the shell sees
# them, so an inline comment would silently swallow every command after it.
RUN set -e; \
    test -x ${PX4_DIR}/build/px4_sitl_default/bin/px4; \
    test -f ${PX4_DIR}/build/px4_sitl_default/build_gazebo-classic/libgazebo_mavlink_interface.so; \
    { cd ${PX4_DIR} && git describe --tags --always; } > /etc/px4-version 2>/dev/null \
        || echo "${PX4_COMMIT}" > /etc/px4-version; \
    set +e; \
    ccache -C >/dev/null 2>&1; \
    find ${PX4_DIR} -maxdepth 3 -name ".git" -exec rm -rf {} + 2>/dev/null; \
    rm -rf ${PX4_DIR}/.ccache /root/.ccache; \
    true

# PX4's setup script installs its own requirements.txt (numpy>=1.13, jinja2, empy, lark...).
# pip leaves a satisfied pin alone, but re-asserting the exact set here makes that guarantee
# explicit: a numpy that drifts to 2.x breaks opencv's ABI and the detector dies on import.
RUN pip3 install --no-cache-dir \
        numpy==1.26.4 opencv-python==4.9.0.80 opencv-contrib-python==4.9.0.80 \
        empy==3.3.4 jinja2==3.1.6 lark==1.1.1

# Assert the vision stack can actually do what perception needs, rather than discovering it
# from a runtime warning buried in swarm.log. A missing CSRT tracker does not crash anything:
# YOLO keeps logging detections while follow silently never locks on, which is the hardest
# kind of failure to trace back to a wheel.
RUN python3 -c "\
import cv2, numpy;\
assert hasattr(cv2, 'TrackerCSRT_create') or hasattr(cv2, 'legacy'), 'no CSRT tracker: install opencv-contrib-python';\
(cv2.TrackerCSRT_create if hasattr(cv2, 'TrackerCSRT_create') else cv2.legacy.TrackerCSRT_create)();\
print('vision OK: cv2', cv2.__version__, '| numpy', numpy.__version__, '| CSRT constructible')"

# --------------------------------------------------------------------------- #
# THIS WORKSTATION'S MODEL AND WORLDS.
#
# Copied AFTER the PX4 build so the upstream iris is replaced, never the other way round.
# The hand-modified iris carries the front camera, downward + depth cameras, 2D/3D lidar and
# sonar this project's perception depends on; the stock upstream model has none of them.
# --------------------------------------------------------------------------- #
# Per-vehicle ROS namespaces for the gazebo_ros sensor plugins. Stock PX4 does not supply the
# `vehicle_namespace` that iris.sdf.jinja expects, so without this every vehicle registers its
# plugins at the root and a 2+ vehicle launch segfaults gzserver on a duplicate node name
# (/slam_imu_plugin), which surfaces as a black Gazebo window. Runs after the firmware build
# because both patched files are runtime scripts - the PX4 compile layer stays cached. The
# script is idempotent and fails the build if PX4 ever moves the lines it anchors to.
COPY context/px4_namespace_patch.py /opt/px4_namespace_patch.py
RUN python3 /opt/px4_namespace_patch.py ${PX4_DIR}

COPY context/models/iris/  ${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/

# Ship ONLY this project's worlds. COPY merges into the directory rather than replacing it,
# so without this clear-out the image also keeps PX4's ~14 upstream worlds and the console
# offers 20 instead of 6 - two of them (safe_landing, yosemite) reference models that were
# never fetched and render with holes in them. Only .world files go; the models directory
# beside it is untouched, and the firmware is already built by this point so nothing that
# runs depends on the files being removed.
RUN rm -f ${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/*.world
COPY context/worlds/       ${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/

# Local Gazebo model cache. The online model database is DISABLED at runtime, so every model a
# world references must already be here or that world renders empty.
COPY context/gazebo_models/ /root/.gazebo/models/

# --------------------------------------------------------------------------- #
# ws_swarm workspace
# --------------------------------------------------------------------------- #
ENV WS=/root/ws_swarm
COPY context/ws_swarm/src/       ${WS}/src/
COPY context/ws_swarm/config/    ${WS}/config/
COPY context/ws_swarm/build.sh   ${WS}/build.sh
RUN chmod +x ${WS}/build.sh
COPY context/ws_swarm/models/    ${WS}/models/
COPY context/ws_swarm/aegis_gcs/ ${WS}/aegis_gcs/

# Use the workspace's OWN build script. swarm_msgs fails under --symlink-install (ament
# tries to symlink over an existing directory), so it must be built plain while everything
# else is symlinked. A bare `colcon build --symlink-install` aborts the image build here.
RUN source /opt/ros/humble/setup.bash && cd ${WS} && ./build.sh

RUN cd ${WS}/aegis_gcs && npm ci --no-audit --no-fund && npm run build

# --------------------------------------------------------------------------- #
# Runtime environment
# --------------------------------------------------------------------------- #
# Offline model database: without this Gazebo blocks for minutes trying to fetch meshes.
ENV GAZEBO_MODEL_DATABASE_URI="" \
    GAZEBO_MODEL_PATH=${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:/root/.gazebo/models \
    GAZEBO_PLUGIN_PATH=${PX4_DIR}/build/px4_sitl_default/build_gazebo-classic \
    FASTRTPS_DEFAULT_PROFILES_FILE=${WS}/config/udp_only.xml \
    ROS_DOMAIN_ID=42 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    NO_PXH=1 \
    PX4_NO_FOLLOW_MODE=1

COPY context/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 8000 8080 9090 5200
WORKDIR ${WS}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["console"]
