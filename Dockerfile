# syntax=docker/dockerfile:1.12.0
FROM osrf/ros:jazzy-desktop-full
ARG USERNAME=racecar
ARG UID=1001
ARG GID=$UID
ARG DEBIAN_FRONTEND=noninteractive
ARG DISPLAY=:0
ARG ROS2_DIR=/ros2_ws
ARG ROS_DISTRO=jazzy

# Create the user
RUN \
groupadd --force --gid $GID $USERNAME && \
useradd --uid $UID --gid $GID -m $USERNAME && \
echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
chmod 0440 /etc/sudoers.d/$USERNAME

# Install dependencies
RUN \
apt-get update && \
apt-get install -y --no-install-recommends software-properties-common && \
add-apt-repository universe && \
apt-get install --no-install-recommends -y \
    curl \
    git \
    tzdata \
    net-tools \
    nmap \
    htop \
    python3-pip \
    python3-rosdep \
    ros-dev-tools && \
apt-get clean

# Configure the timezone
ENV TZ=America/New_York
RUN <<EOF
ln -sf /usr/share/zoneinfo/$TZ /etc/localtime
echo $TZ > /etc/timezone
EOF

# Configure ROS2 workspace and install ROS2 package dependencies
RUN \
mkdir -p $ROS2_DIR/src && \
cd $ROS2_DIR/src && \
apt-get update && \
git clone --branch ros2 --depth 1 https://github.com/RobotWebTools/web_video_server.git && \
git clone --branch ros2 --depth 1 https://github.com/rst-tu-dortmund/costmap_converter.git && \
git clone --branch ros2-master --depth 1 https://github.com/rst-tu-dortmund/teb_local_planner.git && \
git clone --branch ros2 --depth 1 https://github.com/SherbyRobotics/racecar.git && \
cd $ROS2_DIR && \
rosdep update --rosdistro=$ROS_DISTRO && \
rosdep install --rosdistro=$ROS_DISTRO --from-paths src --ignore-src -y && \
apt-get clean

# Clean up cache and unnecessary files to reduce image size
RUN rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN chown --recursive $UID:$GID $ROS2_DIR
COPY --chown=$UID:$GID --chmod=0755 ./colcon.sh $ROS2_DIR/
COPY --chmod=0755 ./ros_entrypoint.sh /
ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD [ "/bin/bash" ]

ENV SHELL=/bin/bash
ENV DISPLAY=$DISPLAY

USER $UID:$GID

WORKDIR $ROS2_DIR
