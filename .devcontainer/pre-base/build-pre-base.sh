#!/bin/bash

docker buildx build . \
    --file pre-base.Dockerfile \
    --tag ghcr.io/ubcsailbot/sailbot_workspace/pre-base:ros_humble-ompl_2db81e2-mongo_367-v2 \
    --platform linux/arm64,linux/amd64 \
    --push
