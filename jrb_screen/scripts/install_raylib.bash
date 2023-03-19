#!/bin/bash

# Exit on error
set -e

# Write a small title of the current action (first param)
function title {
		echo -e "\e[1;36m===== $1 =====\e[0m"
}


title "Installing dependencies"
sudo apt install -y libasound2-dev mesa-common-dev libx11-dev libxrandr-dev libxi-dev xorg-dev libgl1-mesa-dev libglu1-mesa-dev
echo ""

# title "Clone raylib in /tmp"
# git clone https://github.com/raysan5/raylib.git /tmp/raylib
# echo ""

title "Building raylib"
mkdir -p /tmp/raylib/build
cd /tmp/raylib/build
cmake -DBUILD_SHARED_LIBS=ON ..
make
echo ""

title "Installing raybli"
sudo make install
echo ""

title "Finished"

