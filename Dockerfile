# syntax=docker/dockerfile:1

# To test on ARM64 architecture, replace base image with `osrf/ubuntu_arm64:noble`
FROM ubuntu:noble
ARG USERNAME=racecar
ARG USER_UID=1001
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive
ARG DISPLAY=:0
ENV ROS2_DIR=/home/${USERNAME}/ros2_ws

USER root
RUN if id -u $USER_UID ; then groupdel --force `id -un $USER_UID`; userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --force --gid $USER_GID ${USERNAME} && \
    useradd --uid $USER_UID --gid $USER_GID -m ${USERNAME} && \
    apt-get update && \
    apt-get install -y --no-install-recommends sudo && \
    echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

# Configure the timezone
ENV TZ=America/New_York
RUN apt-get update && \
apt-get install -y --no-install-recommends tzdata && \
ln -sf /usr/share/zoneinfo/$TZ /etc/localtime && \
echo $TZ > /etc/timezone && \
apt-get clean

# Clean up cache and unnecessary files to reduce image size
RUN rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

USER ${USERNAME}
WORKDIR /home/${USERNAME}
COPY --chmod=0755 ./images/setup_vm_ubuntu2404_jazzy.bash ./setup_vm_ubuntu2404_jazzy.bash
RUN ./setup_vm_ubuntu2404_jazzy.bash

CMD [ "/bin/bash" ]

ENV SHELL=/bin/bash
ENV DISPLAY=$DISPLAY

WORKDIR $ROS2_DIR
