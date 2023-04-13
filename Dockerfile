FROM arm64v8/ros:humble-perception
SHELL ["/bin/bash", "-c"]

# Create shared directory and ros2 workspace
RUN mkdir -p /shared
RUN mkdir -p ~/ros2_ws/src

# Install ROS2 Humble desktop
RUN sudo apt update && sudo apt install curl gnupg lsb-release -y
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN sudo apt update
RUN sudo apt upgrade -y
RUN sudo apt install ros-humble-desktop -y

# Clone webots_ros2 sources (master branch by default)
RUN source /opt/ros/humble/setup.bash
RUN git clone --recurse-submodules https://github.com/cyberbotics/webots_ros2.git ~/ros2_ws/src/webots_ros2

# Install ros dependencies
RUN sudo apt install python3-pip python3-rosdep python3-colcon-common-extensions -y
RUN sudo rosdep update
RUN sudo apt-get update
RUN rosdep install --from-paths ~/ros2_ws/src --ignore-src --rosdistro humble -y

# Build packages
RUN source /opt/ros/humble/setup.bash && cd ~/ros2_ws && colcon build

# jeroboam deps

# Rosbridge for foxglove
RUN sudo apt install ros-${ROS_DISTRO}-rosbridge-suite -y
RUN sudo apt install ros-${ROS_DISTRO}-turtlebot3-navigation2 -y

# Zsh & oh my zsh
RUN sudo apt install wget zsh -y
RUN wget -O - https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh | sh

# Direnv
RUN sudo apt install vim direnv -y
RUN echo 'eval "$(direnv export bash)"' >> /root/.zshrc

# cyclonedds
RUN sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp -y

EXPOSE 9090/tcp

WORKDIR /root/ros2_ws/src/jeroboam_ros
CMD [ "zsh" ]