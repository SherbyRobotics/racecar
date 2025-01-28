#!/bin/bash

set -eux

IMAGE_NAME="racecar:latest"

echo "Building the $IMAGE_NAME image..."

if sudo docker buildx build --tag ${IMAGE_NAME} . > ./build.log 2>&1; then
    echo "Docker build of $IMAGE_NAME finished successfully."
    rm build.log
    sudo docker image prune -f
else
    echo "Docker build of $IMAGE_NAME failed. Check build.log for details."
fi
