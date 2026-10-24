# Official multi-architecture image; Compose selects native ARM64 on Apple Silicon.
ARG ROS_IMAGE=ros:lyrical-ros-base-resolute
FROM ${ROS_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git gosu sudo \
    python3-colcon-common-extensions python3-rosdep python3-vcstool \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-configs-utils \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    fluxbox novnc websockify x11vnc xvfb x11-utils \
    libgl1-mesa-dri mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Required by the upstream xArm description's standalone visualization helpers.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-joint-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# Build the source-pinned Ouster driver and Gazebo sensor plugin on ARM64.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libpcl-dev libtins-dev libzip-dev \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-foxglove-bridge python3-websocket \
    && rm -rf /var/lib/apt/lists/*

RUN useradd --create-home --uid 1001 --shell /bin/bash ros \
    && echo 'ros ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ros \
    && chmod 0440 /etc/sudoers.d/ros \
    && mkdir -p /ws/build /ws/install /ws/log \
    && chown -R ros:ros /ws

COPY docker/entrypoint.sh docker/ros-shell.sh docker/desktop.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/entrypoint.sh /usr/local/bin/ros-shell.sh /usr/local/bin/desktop.sh \
    && echo 'source /opt/ros/lyrical/setup.bash; if [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi' >> /home/ros/.bashrc

ENV DISPLAY=:1 \
    QT_X11_NO_MITSHM=1 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
WORKDIR /ws
EXPOSE 8765
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
