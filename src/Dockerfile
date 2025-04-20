FROM arm64v8/ros:humble

# Install base system packages
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    python3-pip \
    tree \
    ack \
    vim \
    curl \
    wget \
    zip unzip \
    tar \
    nano \
    git \
    rsync \
    make \
    valgrind \
    tcpdump \
    ssh \
    net-tools \
    gzip \
    jq \
    iputils-ping

# Install Python/ROS development tools
RUN apt-get install -y \
    python3-flake8-docstrings \
    python3-pytest-cov \
    ros-dev-tools \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    ros-${ROS_DISTRO}-rviz2

RUN python -m pip install RPi.GPIO

# Default shell
ENV SHELL=/bin/bash

# Entrypoint script (you must mount or copy this file)
ENTRYPOINT ["/bin/bash", "/robot-arm-slave/src/entrypoint.sh"]