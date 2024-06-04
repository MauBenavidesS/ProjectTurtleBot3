#!/bin/bash
# This script clones the TurtleBot3 repository, initializes and updates submodules, and builds the project

# Source the ROS setup file
source /opt/ros/noetic/setup.bash

# Initialize and update submodules
git submodule init
git submodule update

# Build the project
catkin_make