#!/usr/bin/env bash

IMAGE=horizon_walk_talos

DOCKER_BUILDKIT=1 docker build --build-arg CACHE_DATE="$(date)"--rm -t ${IMAGE} . #--progress=plain --no-cache
    