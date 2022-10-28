FROM ghcr.io/ubcsailbot/sailbot_workspace/dev:infra_updates

## Uncomment this section to install the packages used in the ROS 2 tutorials
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
 && apt-get -y install --no-install-recommends \
 # Your package list here
 ros-eloquent-turtlesim \
 # Clean up
 && apt-get autoremove -y \
 && apt-get clean -y \
 && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=

## Uncomment this section to install additional packages
RUN wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key|sudo apt-key add -
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
 && apt-get -y install --no-install-recommends \
 clang-10 clang-tidy-10 clangd-10 llvm-10 protobuf-compiler libprotobuf-dev \
 && update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-10 100 \
 # Your package list here
 # Clean up
 && apt-get autoremove -y \
 && apt-get clean -y \
 && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=
RUN wget -O boost_1_80_0.tar.gz https://sourceforge.net/projects/boost/files/boost/1.80.0/boost_1_80_0.tar.gz/download \
 && tar xzf boost_1_80_0.tar.gz \
 && cd boost_1_80_0 && ./bootstrap.sh --prefix=/usr/ && ./b2 install \
 && cd .. && rm -rf boost_1_80_0 && rm -f boost_1_80_0.tar.gz

# Copy configuration files (e.g., .vimrc) from config/ to the container's home directory
ARG USERNAME=ros
ARG HOME=/home/$USERNAME
COPY config ${HOME}
