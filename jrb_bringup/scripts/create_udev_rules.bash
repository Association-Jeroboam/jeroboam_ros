#!/bin/bash

# Exit on error
set -e

# Write a small title of the current action (first param)
function title {
		echo -e "\e[1;36m===== $1 =====\e[0m"
}

# cd into parent directory of the script so the relative path are working
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path"

# Arduino rules
title 'Copy rules to /etc/udev/rules.d'
echo "Copy rplidar.rules to /etc/udev/rules.d/"
sudo cp *.rules /etc/udev/rules.d
echo ""

title "Restarting udev"
echo ""

sudo service udev reload
sudo service udev restart

title "Finished"

