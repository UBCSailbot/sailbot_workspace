#!/bin/bash

docker buildx build . \
    --file base.Dockerfile \
    --tag ghcr.io/ubcsailbot/sailbot_workspace/base:ros_humble-ompl_1bb0aa2 \
    --platform linux/arm64/v8,linux/amd64 \
    --push
