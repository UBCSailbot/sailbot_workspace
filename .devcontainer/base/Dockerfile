FROM ros:eloquent-ros-base-bionic
LABEL org.opencontainers.image.source https://github.com/UBCSailbot/sailbot_workspace

# install OMPL
COPY ./install-ompl-ubuntu-python3.sh /ompl/
WORKDIR /ompl
RUN ./install-ompl-ubuntu-python3.sh --python

# clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
WORKDIR /
