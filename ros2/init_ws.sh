#!/usr/bin/bash
set -e

cd ~/ws

# install any missing dependency
sudo apt update
rosdep install --from-paths src --ignore-src -r -y

# initialise and build the ROS workspace
colcon build --symlink-install --event-handler console_direct+
