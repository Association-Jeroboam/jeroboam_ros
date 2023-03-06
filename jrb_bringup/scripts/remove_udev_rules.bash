#!/bin/bash

# Exit on error
set -e

sudo rm /etc/udev/rules.d/arduino.rules
sudo rm /etc/udev/rules.d/rplidar.rules

echo ""
echo "Restarting udev"
echo ""

sudo service udev reload
sudo service udev restart

echo "Finished"

