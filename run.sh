#!/usr/bin/env bash

IMAGE=horizon_walk_docker
NAME=hubble


xhost +local:docker
docker run -it --rm \
	    --privileged \
            --env DISPLAY=$DISPLAY \
            --gpus 'all,"capabilities=compute,utility,graphics"' \
            --env NVIDIA_DRIVER_CAPABILITIES=all \
            --env TERM=xterm-256color \
            --volume /tmp/.X11-unix:/tmp/.X11-unix \
            --volume /dev:/dev \
            --volume /usr/share/glvnd/egl_vendor.d/10_nvidia.json:/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
            --device /dev/dri \
            --name ${NAME} ${IMAGE}
