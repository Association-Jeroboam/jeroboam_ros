#!/bin/bash

# Exit on error
set -e

# Move to the script's directory
SCRIPT_PATH=$(cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P)
cd "$SCRIPT_PATH"

if [ ! -d "./ros2_ws" ]; then
  mkdir -p ./ros2_ws
  chmod 777 ./ros2_ws
fi
touch ./ros2_ws/.dockerignore

echo "Building docker image..."
docker build --platform linux/arm64 -t rpi_cross_compile -f Dockerfile .

echo "Building packages..."
docker run --platform linux/arm64 -it \
  -v $SCRIPT_PATH/ros2_ws:/home/ubuntu/ros2_ws \
  -v $SCRIPT_PATH/../:/home/ubuntu/ros2_ws/src/jeroboam_ros \
  rpi_cross_compile \
  /bin/bash -c "direnv allow && bash"