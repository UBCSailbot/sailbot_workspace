#!/bin/bash

docker buildx build . \
    --file pre-base.Dockerfile \
    --tag ghcr.io/ubcsailbot/pre-base:ros_humble-ompl_4c86b2f \
    --platform linux/arm64,linux/amd64 \
    --push
