# jeroboam_ros

## Installation
### ROS
```shell
# source : https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-galactic-desktop

source /opt/ros/galactic/setup.bash
ros2 run demo_nodes_cpp talker

# In another terminal
source /opt/ros/galactic/setup.bash
ros2 run demo_nodes_py listener
```

### Development tools and ROS tools
```shell
# source: https://docs.ros.org/en/galactic/Installation/Ubuntu-Development-Setup.html#install-development-tools-and-ros-tools
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget

# install some pip packages needed for testing
python3 -m pip install -U \
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
```

### Direnv
```shell
sudo apt install -y direnv
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc
source ~/.bashrc

# In project root
cp .envrc.example .envrc
# Change values like ROS_DOMAIN_ID
direnv allow
```

### Project dependencies
```shell
# Missing keys from rosdep
sudo apt install -y python3-pip
sudo pip3 install transforms3d
sudo apt install -y --no-install-recommends python3-numpy python3-netifaces python3-yaml python3-opencv
sudo apt install -y --no-install-recommends liblog4cxx-dev python3-dev
pip3 install python-can
pip3 install pygame==2.0.0.dev6
sudo apt-get install libSDL2-2.0
sudo apt-get install libSDL2-ttf-2.0.0
sudo apt-get install can-utils
sudo apt install python3-rosdep2
rosdep install -i --from-path src --rosdistro galactic -y

# In project for direnv to load
rosdep_install
```

## Launch
```shell
# In project for direnv to load
ros2 launch jrb_bringup robotrouge.launch.py
```

## Development aliases
```shell
# Build
b

# Clean
c

# Install rosdep dependencies
rosdep_install
```
