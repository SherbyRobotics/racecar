@echo off
setlocal

set IMAGE_NAME=racecar:latest
set CONTAINER_NAME=racecar_container

docker run ^
    --interactive ^
    --tty ^
    --rm ^
    --env ROS2_DIR=/ros2_ws ^
    --publish 10000:10000 ^
    --publish 5005:5005 ^
    --publish 8765:8765 ^
    --privileged ^
    --workdir /ros2_ws ^
    --name %CONTAINER_NAME% ^
    %IMAGE_NAME%

endlocal
