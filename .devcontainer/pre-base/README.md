# Pre-Base Image

## Features

- Builds off the [`ubuntu`](https://hub.docker.com/_/ubuntu) image
- Install ROS Humble
- Installs OMPL with Python bindings
- Builds for both `amd64` and `arm64`

## How to build

### Optimizing build times

Since building OMPL with Python bindings is very CPU and memory intensive, and ARM emulation on x86 is not very fast,
I highly recommend building this image on a bare-metal Linux computer using Docker Engine (not Docker Desktop).
The `-j <num>` argument of `make`/`cmake` parallelizes builds. More parallelization will shorten build times but
use more memory, so you may have to play around with this argument to build on your computer.
For reference, `-j 24` peaks at ~25GB memory and `-j 8` peaks at ~12GB, provided that you have enough logical
processors. If `-j` >= the number of logical processors your computer has, you will be pinned at 100% CPU usage.

### Setup

1. Create a new driver to build this multi-architecture image

   ```
   docker buildx create --name sailbot --platform linux/arm64,linux/amd64 --use
   ```

2. Login to the GitHub container registry: [Authenticating with a personal access token (classic)](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry#authenticating-with-a-personal-access-token-classic)

### Usage

For example, the steps required to update OMPL to the latest commit:

1. In [`pre-base.Dockerfile`](pre-base.Dockerfile), update the commit hash in the `git reset` command and OMPL repository
   URL
2. In [`build-pre-base.sh`](build-pre-base.sh), update the commit hash in the tag
3. In [`base-dev.Dockerfile`](../base-dev/base-dev.Dockerfile), update the commit hash in the pre-base image tag
4. Build and push the image: `./build-pre-base.sh`
5. [Build the base and dev images using the newly built image](../base-dev/README.md#how-to-build)
