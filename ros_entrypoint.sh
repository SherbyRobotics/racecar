#!/bin/bash
set -e

ROS_DISTRO=jazzy

# Setup ROS2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash
exec "$@"