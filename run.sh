#!/usr/bin/env bash

IMAGE=horizon_walk_talos
NAME=hubble

#  --privileged \
#  -v /dev/input:/dev/input \

xhost +local:docker
docker run -it --rm \
            --env DISPLAY=$DISPLAY \
            --runtime=nvidia \
            --gpus all \
            --env NVIDIA_DRIVER_CAPABILITIES=all \
            --env TERM=xterm-256color \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            --name ${NAME} ${IMAGE}
