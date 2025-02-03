#!/usr/bin/env bash

IMAGE=horizon_walk
NAME=hubble

#  --privileged \
#             --runtime=nvidia --gpus all \
#             --env NVIDIA_DRIVER_CAPABILITIES=all \
#             -v /dev/input:/dev/input \
#             -v /tmp/.X11-unix:/tmp/.X11-unix \

docker run -it --rm \
            --env=DISPLAY \
            --name ${NAME} ${IMAGE}
            # --network="host" \
