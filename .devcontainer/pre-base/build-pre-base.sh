#!/bin/bash

docker build . \
    --file base.Dockerfile \
    --tag ghcr.io/ubcsailbot/sailbot_workspace/base:ros_humble-ompl_1bb0aa2-amd64 \
    --build-arg ARCH=amd64/

# docker build . \
#     --file base.Dockerfile \
#     --tag ghcr.io/ubcsailbot/sailbot_workspace/base:ros_humble-ompl_1bb0aa2-arm64v8 \
#     --build-arg ARCH=arm64v8/

# docker buildx build . \
#     --file base.Dockerfile
#     --tag ghcr.io/ubcsailbot/sailbot_workspace/base:ros_humble-ompl_1bb0aa2 \
#     --platform linux/arm64/v8,linux/amd64 \
#     # --push

# docker manifest create ghcr.io/ubcsailbot/sailbot_workspace/base:ros_humble-ompl_1bb0aa2-latest \
#     --amend ghcr.io/ubcsailbot/sailbot_workspace/base:ros_humble-ompl_1bb0aa2-amd64 \
#     --amend ghcr.io/ubcsailbot/sailbot_workspace/base:ros_humble-ompl_1bb0aa2-arm64v8
