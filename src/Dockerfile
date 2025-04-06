FROM arm64v8/ros:humble
#ros:humble
ARG USERNAME=carwyn
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL=/bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

RUN <<EOT

sudo apt update && apt-get upgrade

# DEVELOPER TOOLS

sudo apt install -y tree \
  ack \
  ag \
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
  yq \
  iputils-ping

# ROS2 SETUP

source /opt/ros/humble/setup.bash

sudo apt update && sudo apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools

sudo apt install -y \
   python3-flake8-blind-except \
   python3-flake8-builtins \
   python3-flake8-class-newline \
   python3-flake8-comprehensions \
   python3-flake8-deprecated \
   python3-flake8-import-order \
   python3-flake8-quotes \
   python3-pytest-repeat \
   python3-pytest-rerunfailures

sudo apt install ros-$ROS_DISTRO-rviz2 -y

EOT

COPY ./ros2_ws/ /opt/ros2_ws/
COPY ./entrypoint.sh /opt/run/entrypoint.sh

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME

#CMD ["/bin/bash"]
ENTRYPOINT ["/bin/bash", "/opt/run/entrypoint.sh"]