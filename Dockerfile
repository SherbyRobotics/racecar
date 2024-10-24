FROM ubuntu:jammy
ARG USERNAME=docker
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive
ARG DISPLAY=:0

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
RUN apt-get install -y curl

ENV TZ=America/New_York

# Install the tzdata package to configure the timezone
RUN apt-get update && \
    apt-get install -y tzdata && \
    ln -sf /usr/share/zoneinfo/$TZ /etc/localtime && \
    echo $TZ > /etc/timezone && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

USER $USERNAME
WORKDIR /home/$USERNAME
COPY images/setup_vm_ubuntu2204_humble.bash setup_vm_ubuntu2204_humble.bash
RUN sudo chmod +x setup_vm_ubuntu2204_humble.bash
RUN ./setup_vm_ubuntu2204_humble.bash

ENV SHELL /bin/bash
ENV DISPLAY=$DISPLAY

CMD ["/bin/bash"]