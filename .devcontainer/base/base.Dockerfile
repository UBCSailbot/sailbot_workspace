FROM ubuntu:22.04 AS ompl-source

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y git \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
RUN git clone https://github.com/ompl/ompl.git
WORKDIR /ompl
RUN git reset --hard 1bb0aa2

# From https://github.com/athackst/dockerfiles/blob/32a872348af0ad25ec4a6e6184cb803357acb6ab/ros2/humble.Dockerfile
FROM ubuntu:22.04 AS ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y \
    locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get update \
    && apt-get install -y tzdata \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2
RUN apt-get update && apt-get install -y \
        curl \
        gnupg2 \
        lsb-release \
        sudo \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y \
        ros-humble-ros-base \
        python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV DEBIAN_FRONTEND=

# Based on https://github.com/ompl/ompl/blob/1bb0aa2cae0d5e30eee6efca4a9d10a2da1971dc/scripts/docker/ompl.Dockerfile
FROM ros-base AS ompl-builder
# avoid interactive configuration dialog from tzdata, which gets pulled in
# as a dependency
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y \
        build-essential \
        castxml \
        cmake \
        libboost-all-dev \
        libeigen3-dev \
        libexpat1 \
        libflann-dev \
        libode-dev \
        libtriangle-dev \
        ninja-build \
        pkg-config \
        python3-dev \
        python3-numpy \
        python3-pip \
        pypy3 \
        wget \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
RUN pip3 install pygccxml pyplusplus
COPY --from=ompl-source /ompl /ompl
WORKDIR /build
RUN cmake \
        -DPYTHON_EXEC=/usr/bin/python3 \
        -DOMPL_REGISTRATION=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -G Ninja \
        /ompl \
    && ninja update_bindings -j `nproc` \
    && ninja -j `nproc` \
    && ninja install

FROM ros-base as base
LABEL org.opencontainers.image.source https://github.com/UBCSailbot/sailbot_workspace

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y \
        build-essential \
        cmake \
        libboost-filesystem-dev \
        libboost-numpy-dev \
        libboost-program-options-dev \
        libboost-python-dev \
        libboost-serialization-dev \
        libboost-system-dev \
        libeigen3-dev \
        libflann-dev \
        libode-dev \
        libtriangle-dev \
        ninja-build \
        pkg-config \
        python3-dev \
        python3-numpy \
        python3-pip \
        wget \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
ENV DEBIAN_FRONTEND=

COPY --from=ompl-builder /usr /usr
