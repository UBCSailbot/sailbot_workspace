FROM ubuntu:jammy-20240111 AS fix-certificates

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
ENV DEBIAN_FRONTEND=

FROM fix-certificates AS ompl-source

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends git \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
RUN git clone https://github.com/ompl/ompl.git && cd ompl && git reset --hard 2db81e2

# From https://github.com/athackst/dockerfiles/blob/32a872348af0ad25ec4a6e6184cb803357acb6ab/ros2/humble.Dockerfile
FROM fix-certificates AS ros-pre-base

ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get update \
    && apt-get install -y --no-install-recommends tzdata \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl \
        gnupg2 \
        lsb-release \
        sudo \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
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

# Based on https://github.com/ompl/ompl/blob/2db81e2154cad93f4b823b2afcd3e7c021ea2b2f/scripts/docker/ompl.Dockerfile
FROM fix-certificates AS ompl-builder
# avoid interactive configuration dialog from tzdata, which gets pulled in
# as a dependency
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
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
WORKDIR /ompl
RUN cmake \
        -G Ninja \
        -B build \
        -DPYTHON_EXEC=/usr/bin/python3 \
        -DOMPL_REGISTRATION=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr \
    && cmake --build build -t update_bindings \
    && NPROC=$(nproc) \
    && HALF_NPROC=$((NPROC / 2)) \
    && cmake --build build -- -j $HALF_NPROC \
    && cmake --install build \
    && cd tests/cmake_export \
    && cmake -B build -DCMAKE_INSTALL_PREFIX=../../install \
    && cmake --build build

FROM fix-certificates AS mongo-cxx-driver-builder

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        libmongoc-dev \
        ninja-build \
        wget \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# setup MongoDB C++ Packages
# mongo-cxx-driver version must match libmongoc-dev version - see https://mongocxx.org/mongocxx-v3/installation/linux/
RUN wget https://github.com/mongodb/mongo-cxx-driver/releases/download/r3.6.7/mongo-cxx-driver-r3.6.7.tar.gz \
    && tar -xzf mongo-cxx-driver-r3.6.7.tar.gz \
    && cd mongo-cxx-driver-r3.6.7/build \
    && cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    && NPROC=$(nproc) \
    && HALF_NPROC=$((NPROC / 2)) \
    && cmake --build . -- -j $HALF_NPROC \
    && cmake --build . --target  install
ENV DEBIAN_FRONTEND=

FROM ros-pre-base as pre-base
LABEL org.opencontainers.image.source = "https://github.com/UBCSailbot/sailbot_workspace"

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
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
        libmongoc-dev \
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
COPY --from=mongo-cxx-driver-builder /usr/local /usr/local

FROM pre-base as base

# install base apt dependencies
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        can-utils \
        clang \
        cmake \
        git \
        iproute2 \
        libboost-all-dev \
        libprotobuf-dev \
        protobuf-compiler \
        python3-colcon-common-extensions \
        python3-rosdep \
        tzdata \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/* \
    && rosdep init || echo "rosdep already initialized"
ENV DEBIAN_FRONTEND=

# root bash configuration
ENV ROS_WORKSPACE=/workspaces/sailbot_workspace
COPY update-bashrc.sh /sbin/update-bashrc
RUN chmod +x /sbin/update-bashrc \
    && sync \
    && /bin/bash -c /sbin/update-bashrc \
    && rm /sbin/update-bashrc

# set timezone
ENV TZ="America/Vancouver"

FROM base as local-base

# install virtual iridium dependencies
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python2 \
        socat \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
RUN wget https://bootstrap.pypa.io/pip/2.7/get-pip.py \
    && python2 get-pip.py && pip2 install pyserial && pip2 install requests \
    && rm -f get-pip.py
ENV DEBIAN_FRONTEND=

# set virtual iridium environment variables
ENV LOCAL_TRANSCEIVER_TEST_PORT="/tmp/local_transceiver_test_port"
ENV VIRTUAL_IRIDIUM_PORT="/tmp/virtual_iridium_port"

# downgrade setuptools
# https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/?answer=400052#post-id-400052
RUN pip3 install setuptools==58.2.0

# install MongoDB command line tools
ARG MONGO_TOOLS_VERSION=6.0
RUN . /etc/os-release \
    && curl -sSL "https://www.mongodb.org/static/pgp/server-${MONGO_TOOLS_VERSION}.asc" | sudo apt-key add - \
    && echo "deb [arch=$(dpkg --print-architecture)] http://repo.mongodb.org/apt/ubuntu ${VERSION_CODENAME}/mongodb-org/${MONGO_TOOLS_VERSION} multiverse" | tee /etc/apt/sources.list.d/mongodb-org-${MONGO_TOOLS_VERSION}.list
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        mongodb-database-tools \
        mongodb-mongosh \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
ENV DEBIAN_FRONTEND=

# install rapidyaml for diagnostics
ENV DEBIAN_FRONTEND=noninteractive
RUN wget https://github.com/biojppm/rapidyaml/releases/download/v0.5.0/rapidyaml-0.5.0-src.tgz
RUN tar -xzf rapidyaml-0.5.0-src.tgz && \
    cd rapidyaml-0.5.0-src && \
    cmake -S "." -B ./build/Release/ryml-build "-DCMAKE_INSTALL_PREFIX=/usr" -DCMAKE_BUILD_TYPE=Release && \
    cmake --build ./build/Release/ryml-build --parallel --config Release && \
    cmake --build ./build/Release/ryml-build --config Release --target install && \
    rm -rf *rapidyaml*
ENV DEBIAN_FRONTEND=

FROM local-base as ros-dev

# From https://github.com/athackst/dockerfiles/blob/32a872348af0ad25ec4a6e6184cb803357acb6ab/ros2/humble.Dockerfile
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    bash-completion \
    build-essential \
    cmake \
    gdb \
    git \
    pylint \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    vim \
    wget \
    # Install ros distro testing packages
    ros-humble-ament-lint \
    ros-humble-ament-mypy \
    ros-humble-launch-testing \
    ros-humble-launch-testing-ament-cmake \
    ros-humble-launch-testing-ros \
    python3-autopep8 \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init || echo "rosdep already initialized" \
    # Update pydocstyle
    && pip3 install --upgrade pydocstyle mypy

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # [Optional] Add sudo support for the non-root user
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    # Cleanup
    && rm -rf /var/lib/apt/lists/* \
    && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc
    # Sourcing overlay in update-bashrc.sh instead
    # && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

ENV DEBIAN_FRONTEND=
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

FROM ros-dev as dev

ARG USERNAME=ros
ARG HOME=/home/$USERNAME

# persist ROS logs
RUN mkdir -p ${HOME}/.ros/log \
    && chown -R ${USERNAME} ${HOME}/.ros

# persist bash history
RUN SNIPPET="export PROMPT_COMMAND='history -a' && export HISTFILE=${HOME}/commandhistory/.bash_history" \
    && mkdir ${HOME}/commandhistory \
    && touch ${HOME}/commandhistory/.bash_history \
    && chown ${USERNAME} ${HOME}/commandhistory/.bash_history \
    && echo $SNIPPET >> "${HOME}/.bashrc"

# ros bash configuration
COPY update-bashrc.sh /sbin/update-bashrc
RUN chmod +x /sbin/update-bashrc \
    && chown ros /sbin/update-bashrc \
    && sync \
    && /bin/bash -c /sbin/update-bashrc \
    && rm /sbin/update-bashrc

# plantuml diagram support
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        default-jre graphviz \
    && wget https://github.com/plantuml/plantuml/releases/download/v1.2023.12/plantuml-asl-1.2023.12.jar \
    && mv plantuml-asl-1.2023.12.jar /usr/local/bin/plantuml-asl-1.2023.12.jar \
    && echo "#!/bin/bash\njava -jar /usr/local/bin/plantuml-asl-1.2023.12.jar \$*" > /usr/local/bin/plantuml.sh \
    && chmod +x /usr/local/bin/plantuml.sh \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
ENV DEBIAN_FRONTEND=

ENV PLANTUML_TEMPLATE_PATH="/workspaces/sailbot_workspace/diagrams/template.puml"

# install some clang tools and googletest for network systems
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        clang \
        clangd \
        clang-tidy \
        cmake \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
ENV DEBIAN_FRONTEND=

# install other helpful apt packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        less \
        openssh-client \
        tmux \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
ENV DEBIAN_FRONTEND=

# install dev python3 dependencies
RUN pip3 install \
    # for juypter notebooks
    ipykernel \
