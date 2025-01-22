# syntax=docker/dockerfile:1

# To test on ARM64 architecture, replace base image with `osrf/ubuntu_arm64:noble`
FROM ubuntu:noble
ARG USERNAME=racecar
ARG UID=1001
ARG GID=$UID
ARG DEBIAN_FRONTEND=noninteractive
ARG DISPLAY=:0
ENV ROS2_DIR=/ros2_ws

# Create the user
RUN \
groupadd --force --gid $GID $USERNAME && \
useradd --uid $UID --gid $GID -m $USERNAME && \
apt-get update && \
apt-get install -y --no-install-recommends sudo && \
echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
chmod 0440 /etc/sudoers.d/$USERNAME

# Configure the timezone
ENV TZ=America/New_York
RUN apt-get update && \
apt-get install -y --no-install-recommends tzdata && \
ln -sf /usr/share/zoneinfo/$TZ /etc/localtime && \
echo $TZ > /etc/timezone && \
apt-get clean

# Clean up cache and unnecessary files to reduce image size
RUN rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

COPY --chmod=0755 ./ros_entrypoint.sh /
ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD [ "/bin/bash" ]

ENV SHELL=/bin/bash
ENV DISPLAY=$DISPLAY

USER $UID:$GID

WORKDIR $ROS2_DIR
