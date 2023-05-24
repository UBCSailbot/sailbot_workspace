# Pre-Base Image

## Features

- Builds off the [`ubuntu`](https://hub.docker.com/_/ubuntu) image
- Install ROS Humble
- Installs OMPL with Python bindings
- Builds for both `amd64` and `arm64`

## How to build

### Hardware recommendations

Since building OMPL with Python bindings is very CPU and memory intensive, and ARM emulation on x86 is not very fast,
I highly recommend building this image on a bare-metal Linux computer and giving Docker at least 10GB of memory.
The more CPU cores and memory you can give Docker the better: building this image on an Ubuntu server with
24 cores and 128GB of memory took 20 minutes, peaking at 100% CPU usage and 25GB memory used.

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
