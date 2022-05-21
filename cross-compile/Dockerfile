FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    wget \
    tar \
    python3-pip \
    git \
    python3-numpy \
    sudo \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
    build-essential \
    cmake \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-setuptools \
    python3-testresources \
    && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository universe

# Install ROS2 
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y \
    ros-galactic-ros-base \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    direnv \
    && rm -rf /var/lib/apt/lists/*

# Add user
RUN useradd -m ubuntu
RUN echo "ubuntu:ubuntu" | chpasswd
RUN usermod -aG sudo ubuntu

# ROS2 development dependencies
USER ubuntu
RUN pip3 install --no-cache-dir \
    colcon-common-extensions \
    vcstool \
    lark-parser \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest \
    setuptools
# ENV PATH=/home/ubuntu/.local/bin/:$PATH

# Prepare workspace
COPY bashrc.bash bashrc.bash
RUN cat bashrc.bash >> $HOME/.bashrc 
WORKDIR /home/ubuntu/ros2_ws/src/jeroboam_ros