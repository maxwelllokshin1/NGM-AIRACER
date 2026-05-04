# =============================================================
# AI-Racer-2026 Dockerfile
# Base: ROS2 Humble
# Targets: base, dev, prod
# =============================================================

FROM ros:humble-ros-base AS base

ARG USER_NAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=1000
ARG ROS_DISTRO=humble

ENV LIBGL_ALWAYS_SOFTWARE=1
ENV MESA_GL_VERSION_OVERRIDE=3.3
ENV DEBIAN_FRONTEND=noninteractive

# =============================================================
# 1. System dependencies (root)
# =============================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Core tools
    git \
    git-lfs \
    sudo \
    curl \
    wget \
    unzip \
    ca-certificates \
    coreutils \
    nano \
    pipx \
    # Build tools
    build-essential \
    python3-pip \
    gdb \
    # Networking/debug
    iputils-ping \
    net-tools \
    usbutils \
    openssh-client \
    # Qt/X11 runtime (for GUI passthrough)
    libxcb1 \
    libx11-xcb1 \
    libxcb-glx0 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    xvfb \
    x11vnc \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    # VESC driver deps — must come before colcon build
    ros-humble-asio-cmake-module \
    ros-humble-io-context \
    ros-humble-serial-driver \
    ros-humble-udp-msgs \
    && git lfs install \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# =============================================================
# 2. Create user with passwordless sudo
# =============================================================
RUN groupadd --gid ${USER_GID} ${USER_NAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USER_NAME} \
    && echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} \
    && chmod 0440 /etc/sudoers.d/${USER_NAME}

# =============================================================
# 3. Clone all ROS2 workspace sources (root, chown later)
# =============================================================
RUN mkdir -p /home/${USER_NAME}/ros2_workspaces/src
WORKDIR /home/${USER_NAME}/ros2_workspaces

# micro-ROS
RUN git clone -b ${ROS_DISTRO} \
    https://github.com/micro-ROS/micro_ros_setup.git \
    src/micro_ros_setup

# F1Tenth + ackermann
RUN git clone https://github.com/f1tenth/f1tenth_gym_ros.git src/f1tenth_gym_ros && \
    git clone https://github.com/ros-drivers/ackermann_msgs.git src/ackermann_msgs

# LiDAR — urg_node2 with urg_library as submodule
RUN git clone --recurse-submodules \
    https://github.com/Hokuyo-aut/urg_node2.git \
    src/urg_node2

# Configure LiDAR FOV for UST-10LX (+-12deg = +-0.20944 rad)
RUN cp src/urg_node2/config/params_ether.yaml src/urg_node2/config/ust10lx.yaml && \
    sed -i 's/angle_min.*/angle_min: -0.20944/' src/urg_node2/config/ust10lx.yaml && \
    sed -i 's/angle_max.*/angle_max: 0.20944/' src/urg_node2/config/ust10lx.yaml

# F1tenth stack
RUN git clone --recurse-submodules https://github.com/f1tenth/f1tenth_system.git src/f1tenth_system

# FIX: default vesc submodule in f1tenth_system is outdated on Humble
RUN rm -rf src/f1tenth_system/vesc && \
    git clone -b ros2 https://github.com/f1tenth/vesc.git src/f1tenth_system/vesc

# Fix f1tenth_gym_ros map path
RUN sed -i "s|map_path: .*|map_path: '/home/${USER_NAME}/ros2_workspaces/src/f1tenth_gym_ros/maps/levine'|g" \
    src/f1tenth_gym_ros/config/sim.yaml

# =============================================================
# 3b. Patch vesc.yaml in-place for D3542 1450KV + DS3240 40kg
#     Uses sed so the upstream file structure is preserved.
#     All values match vesc.yaml generated for this hardware.
# =============================================================
RUN VESC_YAML=src/f1tenth_system/f1tenth_stack/config/vesc.yaml && \
    # --- BLDC: D3542 1450KV, 7 pole pairs (14-pole motor) ---
    sed -i 's/speed_to_erpm_gain:.*/speed_to_erpm_gain: 10150.0/'       $VESC_YAML && \
    sed -i 's/speed_to_erpm_offset:.*/speed_to_erpm_offset: 0.0/'       $VESC_YAML && \
    # --- Servo: DS3240 40kg, 500-2500us, neutral=1500us ---
    # gain = 0.5 / 0.524 rad (+-30deg assumed throw) — re-measure and retune
    sed -i 's/steering_angle_to_servo_gain:.*/steering_angle_to_servo_gain: -0.9549/'   $VESC_YAML && \
    sed -i 's/steering_angle_to_servo_offset:.*/steering_angle_to_servo_offset: 0.5/'   $VESC_YAML && \
    # --- Serial port ---
    sed -i 's|port:.*|port: /dev/ttyACM0|'                              $VESC_YAML && \
    # --- Current limits ---
    sed -i 's/current_max:.*/current_max: 100.0/'                       $VESC_YAML && \
    # --- Speed limits: 10150 * 2.5 m/s = 25375 erpm ---
    sed -i 's/speed_min:.*/speed_min: -25375.0/'                        $VESC_YAML && \
    sed -i 's/speed_max:.*/speed_max: 25375.0/'                         $VESC_YAML && \
    # --- Servo range: DS3240 wider PWM range, protect hard stops ---
    sed -i 's/servo_min:.*/servo_min: 0.10/'                            $VESC_YAML && \
    sed -i 's/servo_max:.*/servo_max: 0.90/'                            $VESC_YAML && \
    # --- Odometry ---
    sed -i 's/wheelbase:.*/wheelbase: 0.25/'                            $VESC_YAML

# =============================================================
# 4. rosdep install
# =============================================================
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Fix ownership before switching user
RUN chown -R ${USER_UID}:${USER_GID} /home/${USER_NAME}

# =============================================================
# 5. Build workspace as user
# =============================================================
USER ${USER_NAME}

RUN rosdep update

# Build full workspace
RUN /bin/bash -c "\
    . /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /home/${USER_NAME}/ros2_workspaces && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log"

# Build micro-ROS agent
RUN /bin/bash -c "\
    . /opt/ros/${ROS_DISTRO}/setup.bash && \
    . /home/${USER_NAME}/ros2_workspaces/install/setup.bash && \
    cd /home/${USER_NAME}/ros2_workspaces && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh && \
    rm -rf microros_agent_ws/build microros_agent_ws/log"

# =============================================================
# 6. Shell environment
# =============================================================
RUN echo "" >> /home/${USER_NAME}/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USER_NAME}/.bashrc && \
    echo "source /home/${USER_NAME}/ros2_workspaces/install/setup.bash" >> /home/${USER_NAME}/.bashrc && \
    echo 'function vnc() { bash ~/scripts/start_vnc.sh && export DISPLAY=:99; }' >> /home/${USER_NAME}/.bashrc

# =============================================================
# 7. PlatformIO (for embedded firmware flashing)
# =============================================================
ENV PATH="/home/${USER_NAME}/.local/bin:$PATH"
RUN pipx install platformio && \
    pipx inject platformio pyyaml && \
    pipx ensurepath

# =============================================================
# 8. Python + GPIO libraries (root)
# =============================================================
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-serial \
    python3-requests \
    python3-gpiozero \
    python3-lgpio \
    && pip3 install pyusb \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# =============================================================
# DEV target — RViz2, Nav2, Foxglove, F1Tenth gym, GUI tools
# =============================================================
FROM base AS dev

USER root

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rviz2 \
    ros-humble-rqt-graph \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-xacro \
    ros-humble-foxglove-bridge \
    python3-tk \
    x11-apps \
    xvfb \
    x11vnc \
    fluxbox \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# yq — ARM64 binary for Jetson TX2
RUN wget -q https://github.com/mikefarah/yq/releases/latest/download/yq_linux_arm64 \
    -O /usr/bin/yq && chmod +x /usr/bin/yq

# F1Tenth gym physics engine
RUN git clone https://github.com/f1tenth/f1tenth_gym.git /opt/f1tenth_gym && \
    pip3 install --no-cache-dir --default-timeout=120 -e /opt/f1tenth_gym

USER ${USER_NAME}
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# =============================================================
# PROD target — minimal runtime only
# =============================================================
FROM base AS prod

USER ${USER_NAME}
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
