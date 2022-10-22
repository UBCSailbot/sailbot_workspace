FROM althack/ros2:humble-dev 

ARG USERNAME=ros
ARG HOME=/home/$USERNAME

# persist ROS logs
RUN mkdir -p ${HOME}/.ros/log \
    && chown -R ${USERNAME} ${HOME}/.ros

# persist bash history
RUN SNIPPET="export PROMPT_COMMAND='history -a' && export HISTFILE=${HOME}/.commandhistory/.bash_history" \
    && mkdir ${HOME}/.commandhistory \
    && touch ${HOME}/.commandhistory/.bash_history \
    && chown ${USERNAME} ${HOME}/.commandhistory/.bash_history \
    && echo $SNIPPET >> "${HOME}/.bashrc"

ARG ROS_WORKSPACE=/workspaces/sailbot_workspace

# bash configuration
COPY update-bashrc.sh /sbin/update-bashrc
RUN chmod +x /sbin/update-bashrc ; chown ros /sbin/update-bashrc ; sync ; /bin/bash -c /sbin/update-bashrc ; rm /sbin/update-bashrc

# install git-delta
RUN wget https://github.com/dandavison/delta/releases/download/0.14.0/git-delta-musl_0.14.0_amd64.deb
RUN dpkg -i git-delta-musl_0.14.0_amd64.deb
RUN rm git-delta-musl_0.14.0_amd64.deb

# install additional apt packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get -y install --no-install-recommends \
    # Your package list here
    less \
    tmux \
    # Clean up
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=
