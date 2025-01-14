#!/usr/bin/env bash

IMAGE=horizon_walk

docker build --build-arg CACHE_DATE="$(date)"--rm -t ${IMAGE} . #--progress=plain --no-cache
