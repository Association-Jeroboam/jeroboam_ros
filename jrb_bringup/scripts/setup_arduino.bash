#!/bin/bash

# Exit on error
set -e

echo "Add user to dialout group in order to avoid sudo for serial related commands"
sudo usermod -a -G dialout $USER
echo ""

echo "Add ~/bin to PATH in .bashrc"
echo 'export PATH=$PATH:~/bin' >> ~/.bashrc
source ~/.bashrc
echo ""

echo "Download and install arduino-cli"
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
echo ""

echo "Setup arduino-cli"
arduino-cli core update-index
echo ""

echo "List boards"
arduino-cli board list

echo "Finished"