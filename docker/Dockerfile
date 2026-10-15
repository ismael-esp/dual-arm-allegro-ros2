# ROS 2 Jazzy (Ubuntu 24.04) image for the dual-arm UR3/UR3e + Allegro V4 + RealSense cell.
# Contains every apt package the drivers need plus the PEAK PCAN user-space libraries.
# The workspace itself is NOT baked in: it is mounted at /home/dev/ros2_ws (see docker/run.sh),
# because the launch files reference config files under that path.
FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --- ROS packages: ros2_control, UR driver deps, MoveIt 2, RealSense SDK, Cyclone DDS ---
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
      ros-jazzy-controller-manager ros-jazzy-joint-state-broadcaster \
      ros-jazzy-joint-trajectory-controller ros-jazzy-position-controllers \
      ros-jazzy-velocity-controllers ros-jazzy-force-torque-sensor-broadcaster \
      ros-jazzy-pose-broadcaster ros-jazzy-ur-client-library ros-jazzy-ur-msgs \
      ros-jazzy-moveit ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui \
      ros-jazzy-librealsense2 ros-jazzy-diagnostic-updater ros-jazzy-backward-ros \
      ros-jazzy-launch-pytest ros-jazzy-generate-parameter-library \
      python3-vcstool python3-colcon-common-extensions python3-rosdep \
      python3-numpy python3-scipy python3-zmq python3-pip \
      libyaml-cpp-dev libeigen3-dev libpopt-dev libelf-dev wget git nano iputils-ping net-tools \
    && rm -rf /var/lib/apt/lists/*

# --- build deps of the Allegro library and tools ---
RUN apt-get update && apt-get install -y --no-install-recommends \
      libtinyxml-dev libboost-filesystem-dev libsocketcan-dev libpcap-dev liburdfdom-tools \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages ikpy==3.4.2

# --- PEAK PCAN user-space libraries (libpcan / libpcanfd / libpcanbasic) for the Allegro hands ---
# The pcan kernel module itself must be built and loaded on the HOST (scripts/install_pcan_host.sh).
ARG PEAK_VER=8.20.0
ARG PEAK_SHA256=efecadba9294edc38bbe189387708d0c34b25d7621fbfd4208d8b12811fc2580
RUN cd /tmp && wget -q https://www.peak-system.com/fileadmin/media/linux/files/peak-linux-driver-${PEAK_VER}.tar.gz \
    && echo "${PEAK_SHA256}  peak-linux-driver-${PEAK_VER}.tar.gz" | sha256sum -c - \
    && tar xzf peak-linux-driver-${PEAK_VER}.tar.gz && cd peak-linux-driver-${PEAK_VER} \
    && make -C lib && make -C lib install \
    && make -C libpcanbasic && make -C libpcanbasic install \
    && install -m 644 driver/pcan.h driver/pcanfd.h /usr/include/ \
    && ldconfig && cd / && rm -rf /tmp/peak-linux-driver-*

# --- DDS settings shared with the host (ROS_DOMAIN_ID 10, Cyclone, no shared memory) ---
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_DOMAIN_ID=10 \
    RMW_CYCLONEDDS_USE_ICEORYX=0 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
    CYCLONEDDS_URI=file:///home/dev/ros2_ws/.config/cyclonedds/cyclonedds.xml

RUN mkdir -p /home/dev && chmod 777 /home/dev \
    && echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc \
    && echo '[ -f /home/dev/ros2_ws/install/setup.bash ] && source /home/dev/ros2_ws/install/setup.bash' >> /etc/bash.bashrc
ENV HOME=/home/dev
WORKDIR /home/dev/ros2_ws
CMD ["bash"]
