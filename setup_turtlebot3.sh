#!/bin/bash
# This script clones the TurtleBot3 repository, initializes and updates submodules, and builds the project

# Source the ROS setup file
source /opt/ros/noetic/setup.bash

# Create Symbolic Link between python and python3
sudo ln -sf /usr/bin/python3 /usr/bin/python 
# To remove the symbolic link run the following command: 
#         unlink /usr/bin/python

# Initialize and update submodules
git submodule init
git submodule update

# Build the project
catkin_make
