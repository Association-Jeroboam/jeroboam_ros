# Copyright 2020-2022 Tiryoh<tiryoh@gmail.com>
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

FROM tiryoh/ubuntu-desktop-lxde-vnc:jammy
LABEL maintainer="Tiryoh<tiryoh@gmail.com>"

SHELL ["/bin/bash", "-c"]

RUN apt-get update -q && \
    apt-get upgrade -yq && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends wget curl git build-essential vim sudo lsb-release locales bash-completion tzdata gosu terminator && \
    rm -rf /var/lib/apt/lists/*
RUN rm /etc/apt/apt.conf.d/docker-clean
RUN useradd --create-home --home-dir /home/ubuntu --shell /bin/bash --user-group --groups adm,sudo ubuntu && \
    echo ubuntu:ubuntu | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ARG ROS_DISTRO=humble
ARG INSTALL_PACKAGE=desktop

RUN apt-get update -q && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update -q && \
    apt-get install -y ros-${ROS_DISTRO}-${INSTALL_PACKAGE} \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep python3-vcstool && \
    rosdep init && \
    rm -rf /var/lib/apt/lists/*

RUN gosu ubuntu rosdep update && \
    grep -F "source /opt/ros/${ROS_DISTRO}/setup.bash" /home/ubuntu/.bashrc || echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/ubuntu/.bashrc && \
    grep -F "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" /home/ubuntu/.bashrc || echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/ubuntu/.bashrc && \
    sudo chown ubuntu:ubuntu /home/ubuntu/.bashrc

ENV USER ubuntu

# Create shared directory and ros2 workspace
RUN mkdir -p /shared
RUN mkdir -p ~/ros2_ws/src

# Clone webots_ros2 sources (master branch by default)
RUN source /opt/ros/humble/setup.bash
RUN git clone --recurse-submodules https://github.com/cyberbotics/webots_ros2.git ~/ros2_ws/src/webots_ros2

# Install ros dependencies
RUN sudo apt-get update
RUN sudo apt-get install python3-pip python3-rosdep python3-colcon-common-extensions -y
RUN rosdep update --rosdistro $ROS_DISTRO && rosdep install --from-paths /home/ubuntu/ros2_ws/src --ignore-src --rosdistro humble -y

# Build packages
RUN source /opt/ros/humble/setup.bash && cd ~/ros2_ws && colcon build

# jeroboam deps

# Rosbridge for foxglove
RUN sudo apt-get install ros-${ROS_DISTRO}-rosbridge-suite -y
RUN sudo apt-get install ros-${ROS_DISTRO}-turtlebot3-navigation2 -y

# Zsh & oh my zsh
RUN sudo apt install wget zsh -y
RUN wget -O - https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh | sh
RUN chsh -s $(which zsh)
# Direnv
RUN sudo apt install vim direnv -y
RUN echo 'eval "$(direnv export zsh)"' >> /home/ubuntu/.zshrc

# cyclonedds
RUN sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp -y
