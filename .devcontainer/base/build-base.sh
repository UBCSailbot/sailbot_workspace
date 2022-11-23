#!/bin/bash

git clone https://github.com/ompl/ompl.git
pushd ompl
git reset --hard d8375d842a7f016d64f27a81d6f95eaa83fbe595
popd

docker build . \
    --file base.Dockerfile \
    --tag ghcr.io/ubcsailbot/sailbot_workspace/base:ros_humble-ompl_d8375d8-amd64 \
    --build-arg ARCH=amd64/

# docker build . \
#     --file base.Dockerfile \
#     --tag ghcr.io/ubcsailbot/sailbot_workspace/base:ros_humble-ompl_d8375d8-arm64v8 \
#     --build-arg ARCH=arm64v8/

# docker buildx build . \
#     --file base.Dockerfile
#     --tag ghcr.io/ubcsailbot/sailbot_workspace/base:ros_humble-ompl_d8375d8 \
#     --platform linux/arm64/v8,linux/amd64 \
#     # --push

# docker manifest create ghcr.io/ubcsailbot/sailbot_workspace/base:ros_humble-ompl_d8375d8-latest \
#     --amend ghcr.io/ubcsailbot/sailbot_workspace/base:ros_humble-ompl_d8375d8-amd64 \
#     --amend ghcr.io/ubcsailbot/sailbot_workspace/base:ros_humble-ompl_d8375d8-arm64v8

rm -rf ompl
