#!/bin/bash

docker build . \
    --file dev.Dockerfile \
    --tag ghcr.io/ubcsailbot/sailbot_workspace/dev:ros_humble-ompl_d8375d8-amd64 \
    --build-arg ARCH=amd64/
