#!/bin/bash

docker buildx build . \
    --file base.Dockerfile \
    --tag ghcr.io/ubcsailbot/sailbot_workspace/base:ros_humble-ompl_4c86b2f \
    --platform linux/arm64,linux/amd64 \
    --push
