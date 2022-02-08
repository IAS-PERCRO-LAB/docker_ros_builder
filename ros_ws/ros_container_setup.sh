#!/usr/bin/bash
set -e

cd ~/catkin_ws

# install any missing dependency
sudo apt update
rosdep install --from-paths src --ignore-src -r -y

# initialise and build the ROS workspace
catkin init
catkin config --merge-devel
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
