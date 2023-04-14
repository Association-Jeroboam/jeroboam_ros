#!/bin/bash

# Exit on error
set -e

# Write a small title of the current action (first param)
function title {
		echo -e "\e[1;36m===== $1 =====\e[0m"
}

title "Removing rules from /etc/udev/rules.d"
sudo rm /etc/udev/rules.d/99-candlelight.rules
sudo rm /etc/udev/rules.d/100-arduino.rules
sudo rm /etc/udev/rules.d/101-rplidar.rules
sudo rm /etc/udev/rules.d/102-camera.rules
sudo rm /etc/udev/rules.d/103-can-txqueuelen.rules
echo ""

title "Restarting udev"
sudo service udev reload
sudo service udev restart
echo ""

title "Finished"

