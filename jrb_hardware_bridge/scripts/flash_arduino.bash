#!/bin/bash

# Exit on error
set -e

# cd into parent directory of the script so the relative path are working
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path"

arduino-cli compile -e --fqbn arduino:avr:leonardo ../src/firmware -v
arduino-cli upload -p $(readlink -f /dev/arduino)  --fqbn arduino:avr:leonardo ../src/firmware -v
