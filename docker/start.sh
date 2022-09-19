#!/bin/bash

docker run --rm -it -p 14900:14900 -p 14901:14901 -p 14970:14970 -p 14971:14971 --env="DISPLAY=host.docker.internal:0" -v /tmp/.X11-unix:/tmp/.X11-unix:rw pla10/ros2_humble:arm64 /bin/bash