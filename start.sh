#!/bin/bash

xhost +
docker run --rm -it --name ros2 --network host --env="DISPLAY=host.docker.internal:0" -v /tmp/.X11-unix:/tmp/.X11-unix:rw pla10/ros2_humble:amd64 /bin/bash