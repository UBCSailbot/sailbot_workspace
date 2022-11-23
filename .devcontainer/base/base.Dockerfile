# Based on https://github.com/ompl/ompl/blob/d8375d842a7f016d64f27a81d6f95eaa83fbe595/scripts/docker/ompl.Dockerfile
FROM ubuntu:jammy AS builder
# avoid interactive configuration dialog from tzdata, which gets pulled in
# as a dependency
ENV DEBIAN_FRONTEND=noninteractive
# alternative method of installing libspot
RUN apt-get update && \
    apt-get install -y \
        software-properties-common && \
    add-apt-repository ppa:asiffer/libspot
RUN apt-get update && \
    apt-get install -y \
        build-essential \
        castxml \
        cmake \
        libboost-filesystem-dev \
        libboost-numpy-dev \
        libboost-program-options-dev \
        libboost-python-dev \
        libboost-serialization-dev \
        libboost-system-dev \
        libboost-test-dev \
        libeigen3-dev \
        libexpat1 \
        libflann-dev \
        libode-dev \
        libspot-dev \
        libtriangle-dev \
        ninja-build \
        pkg-config \
        python3-dev \
        python3-numpy \
        python3-pip \
        pypy3 \
        wget && \
    pip3 install pygccxml pyplusplus
COPY ompl /ompl
WORKDIR /build
RUN cmake \
        -DPYTHON_EXEC=/usr/bin/python3 \
        -DOMPL_REGISTRATION=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -G Ninja \
        /ompl && \
    ninja update_bindings -j `nproc` && \
    ninja -j `nproc` && \
    ninja install

FROM ubuntu:jammy
ENV DEBIAN_FRONTEND=noninteractive
# alternative method of installing libspot
RUN apt-get update && \
    apt-get install -y \
        software-properties-common && \
    add-apt-repository ppa:asiffer/libspot
RUN apt-get update && \
    apt-get install -y \
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
        libspot-dev \
        libtriangle-dev \
        ninja-build \
        pkg-config \
        python3-dev \
        python3-numpy \
        python3-pip \
        wget

COPY --from=builder /usr /usr

RUN apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*
