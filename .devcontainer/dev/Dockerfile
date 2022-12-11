# build using Build Dev Image workflow
FROM ghcr.io/ubcsailbot/sailbot_workspace/base:raye as builder

# remove deprecated PPA from base:raye image
# TODO: remove this section once base:raye is rebuilt
RUN sed -i '$d' /etc/apt/sources.list

# develop configuration based on athackst's image
# https://github.com/athackst/dockerfiles/blob/main/ros2/eloquent.Dockerfile
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
  bash-completion \
  build-essential \
  cmake \
  curl \
  gdb \
  git \
  pylint3 \
  python3-argcomplete \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  vim \
  wget \
  # Install ros distro testing packages
  ros-eloquent-ament-lint \
  ros-eloquent-launch-testing \
  ros-eloquent-launch-testing-ament-cmake \
  ros-eloquent-launch-testing-ros \
  python-autopep8 \
  # Packages not from athackst's image
  ros-eloquent-rqt* \
  ros-eloquent-ros2bag \
  ros-eloquent-rosbag2-converter-default-plugins \
  ros-eloquent-rosbag2-storage-default-plugins \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized" \
  # Update pydocstyle
  && pip install --upgrade pydocstyle

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
  # && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

ENV DEBIAN_FRONTEND=
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

FROM builder

# install latest version of git and other related packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
  && apt-get -y install --no-install-recommends software-properties-common less tmux openssh-client \
  && add-apt-repository ppa:git-core/ppa -y \
  && apt-get update \
  && apt-get -y install --no-install-recommends git \
  # Clean up
  && apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=

# install clang, some clang tools, and protobuf for network systems
RUN wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key|sudo apt-key add -
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
  && apt-get -y install --no-install-recommends \
  clang-10 clang-tidy-10 clangd-10 llvm-10 protobuf-compiler libprotobuf-dev \
  && update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-10 100 \
  # Clean up
  && apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=

# build Boost from source to get the latest version
RUN wget -O boost_1_80_0.tar.gz https://sourceforge.net/projects/boost/files/boost/1.80.0/boost_1_80_0.tar.gz/download \
 && tar xzf boost_1_80_0.tar.gz \
 && cd boost_1_80_0 && ./bootstrap.sh --prefix=/usr/ && ./b2 install \
 && cd .. && rm -rf boost_1_80_0 && rm -f boost_1_80_0.tar.gz

# build and install googletest
RUN git clone https://github.com/google/googletest.git -b release-1.12.1 \
 && cd googletest && mkdir build && cd build && cmake .. -DBUILD_GMOCK=OFF \
 && make && make install && cd ../../ && rm -r googletest

# install git-delta
RUN wget https://github.com/dandavison/delta/releases/download/0.14.0/git-delta-musl_0.14.0_amd64.deb
RUN dpkg -i git-delta-musl_0.14.0_amd64.deb
RUN rm git-delta-musl_0.14.0_amd64.deb

ARG USERNAME=ros
ARG HOME=/home/$USERNAME

# install and setup starship
RUN curl -sS https://starship.rs/install.sh -o install_starship.sh \
  && sh install_starship.sh -y \
  && rm install_starship.sh
ENV STARSHIP_CONFIG=/workspaces/sailbot_workspace/.devcontainer/config/starship.toml
#ENV STARSHIP_CACHE=
RUN echo 'eval "$(starship init bash)"' >> ${HOME}/.bashrc

# persist ROS logs
RUN mkdir -p ${HOME}/.ros/log \
    && chown -R ${USERNAME} ${HOME}/.ros

# persist bash history
RUN SNIPPET="export PROMPT_COMMAND='history -a' && export HISTFILE=${HOME}/commandhistory/.bash_history" \
  && mkdir ${HOME}/commandhistory \
  && touch ${HOME}/commandhistory/.bash_history \
  && chown ${USERNAME} ${HOME}/commandhistory/.bash_history \
  && echo $SNIPPET >> "${HOME}/.bashrc"

ARG ROS1_WORKSPACE=/workspaces/catkin_ws
ARG ROS2_WORKSPACE=/workspaces/sailbot_workspace

# bash configuration
COPY update-bashrc.sh /sbin/update-bashrc
RUN chmod +x /sbin/update-bashrc ; chown ros /sbin/update-bashrc ; sync ; /bin/bash -c /sbin/update-bashrc ; rm /sbin/update-bashrc

# ROS 1 workspace setup
WORKDIR $ROS1_WORKSPACE
RUN mkdir src
RUN git clone https://github.com/UBCSailbot/raye-local-pathfinding.git src/local-pathfinding
RUN git clone https://github.com/UBCSailbot/raye-boat-controller.git src/boat_controller
RUN git clone https://github.com/UBCSailbot/raye-sailbot-msg.git src/sailbot-msg
RUN /bin/bash -c "source /opt/ros/${ROS1_DISTRO}/setup.bash; catkin_make"
RUN chown -R ${USERNAME} .

# ROS 2 workspace setup
WORKDIR /
