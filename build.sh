#!/usr/bin/env bash

IMAGE=horizon_walk

DOCKER_BUILDKIT=1 docker build --ssh default --build-arg CACHE_DATE="$(date)"--rm -t ${IMAGE} . #--progress=plain --no-cache
    