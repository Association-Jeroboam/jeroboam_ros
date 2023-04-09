#!/bin/bash

# Exit on error
set -e

# Write a small title of the current action (first param)
function title {
		echo -e "\e[1;36m===== $1 =====\e[0m"
}

title "Add user to dialout group in order to avoid sudo for serial related commands"
sudo usermod -a -G dialout $USER
echo ""

title "Add ~/bin to PATH in .bashrc"
echo 'export PATH=$PATH:~/bin' >> ~/.bashrc
source ~/.bashrc
echo ""

title "Download and install arduino-cli"
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
echo ""

title "Setup arduino-cli"
arduino-cli core update-index
echo ""

title "Setup config with unsafe install"
arduino-cli config init --overwrite
echo "board_manager:
  additional_urls: []
build_cache:
  compilations_before_purge: 10
  ttl: 720h0m0s
daemon:
  port: "50051"
directories:
  data: /home/jeroboam/.arduino15
  downloads: /home/jeroboam/.arduino15/staging
  user: /home/jeroboam/Arduino
library:
  enable_unsafe_install: true
logging:
  file: ""
  format: text
  level: info
metrics:
  addr: :9090
  enabled: true
output:
  no_color: false
sketch:
  always_export_binaries: false
updater:
  enable_notification: true" > ~/.arduino15/arduino-cli.yaml

title "List boards"
arduino-cli board list
echo ""

title "Install libraries"
arduino-cli lib install --git-url https://github.com/adafruit/Adafruit_NeoPixel.git
echo ""

title "Finished"