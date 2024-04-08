#!/bin/bash

# Run this script to install the robot launcher desktop application.

# get the path to the directory containing this script
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# link robot launcher to a location on the PATH
sudo ln -sf "$SCRIPT_DIR/robot_launcher.sh" "/usr/bin/robot_launcher.sh"
# link the launch robots desktop application
sudo ln -sf "$SCRIPT_DIR/launch_robots.desktop" "/usr/share/applications/launch_robots.desktop"
# link the png to /usr/share/pixmaps
sudo ln -sf "$SCRIPT_DIR/../icons/botbuilt_icon.png" "/usr/share/pixmaps/botbuilt_icon.png"

# install the application so it is found by desktop search
xdg-desktop-menu install /usr/share/applications/launch_robots.desktop --novendor
