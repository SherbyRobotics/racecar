# syntax=docker/dockerfile:1

# To test on ARM64 architecture, replace base image with `osrf/ubuntu_arm64:noble`
FROM ubuntu:noble
ENV USER=racecar
ARG UID=1001
ARG GID=$UID
ARG DEBIAN_FRONTEND=noninteractive
ARG DISPLAY=:0
ENV ROS2_DIR=/home/${USER}/ros2_ws

# Create the user
RUN \
groupadd --force --gid $GID ${USER} && \
useradd --uid $UID --gid $GID -m ${USER} && \
apt-get update && \
apt-get install -y --no-install-recommends sudo && \
echo ${USER} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USER} && \
chmod 0440 /etc/sudoers.d/${USER}

# Configure the timezone
ENV TZ=America/New_York
RUN apt-get update && \
apt-get install -y --no-install-recommends tzdata && \
ln -sf /usr/share/zoneinfo/$TZ /etc/localtime && \
echo $TZ > /etc/timezone && \
apt-get clean

# Clean up cache and unnecessary files to reduce image size
RUN rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

USER ${USER}
COPY --chmod=0755 ./images/setup_vm_ubuntu2404_jazzy.sh /home/${USER}/setup_vm_ubuntu2404_jazzy.sh
RUN /home/${USER}/setup_vm_ubuntu2404_jazzy.sh

CMD [ "/bin/bash" ]

ENV SHELL=/bin/bash
ENV DISPLAY=$DISPLAY

WORKDIR $ROS2_DIR
