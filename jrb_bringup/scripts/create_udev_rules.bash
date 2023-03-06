#!/bin/bash

# Exit on error
set -e

# cd into parent directory of the script so the relative path are working
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path"

# Arduino rules
echo "Remap the integrated arduino serial port (/dev/ttyACMX) to /dev/arduino"
echo "Copy rplidar.rules to /etc/udev/rules.d/"
sudo cp arduino.rules  /etc/udev/rules.d

echo ""

# Lidar rules
echo "Remap lidar serial port (/dev/ttyUSBX) to /dev/rplidar"
echo "Copy rplidar.rules to /etc/udev/rules.d/"
sudo cp rplidar.rules  /etc/udev/rules.d

echo ""
echo "Restarting udev"
echo ""

sudo service udev reload
sudo service udev restart

echo "Finished"

