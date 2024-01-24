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
For reference, building with 24 cores uses up to 25GB memory and finishes in 20 minutes, and
building with 8 uses up to 12GB and finishes in 40 minutes, without bottlenecks.
By default building is done with half the number of jobs as logical processors (`HALF_NPROC`) to avoid memory
bottlenecks, but if your computer has a lot of memory you can set `-j $NPROC` to minimize build times.

### Setup

1. Create a new driver to build this multi-architecture image

   ```
   docker buildx create --name sailbot --platform linux/arm64,linux/amd64
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

### Debugging

If you are getting build errors, try:

1. Commenting out the lines that cause, and that are after, the error
2. Building a test image: `docker build -f pre-base.Dockerfile -t pre-base-debug`
3. Running the commented out commands interactively: `docker run -it --rm pre-base-debug`
4. Inspecting the container for things that went wrong
