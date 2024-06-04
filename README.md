# TurtleBot3 Setup and Usage Guide

## Introduction

This guide provides instructions on setting up and using the TurtleBot3 robot.

### Prerequisites

Before getting started, ensure you have the following:

* Ubuntu installed on your system
* Git installed (`sudo apt-get install git`)
* ROS (Robot Operating System) installed on your system

## Installation

To install TurtleBot3, follow these steps:

1. Clone the TurtleBot3 repository:
   ```bash
   git clone https://github.com/MauBenavidesS/ProjectTurtleBot3.git
2. Navigate to the TurtleBot3 directory:
    ```bash
    cd ProjectTurtleBot3/
3. Execute Setup Shell Script to clone submodules and build package:
    ```bash
    sudo ./setup_turtlebot3.sh
4. Source package's setup.bash
    ```bash
    source ./devel/setup.bash
## Execute TurtleBot3
1. Launch turtlebot3_world.launch
    ```bash
    roslaunch turtlebot3_gazebo turtlebot3_world.launch
2. Launch turtlebot3_avoidance.launch
    ```bash
    roslaunch turtlebot3_avoidance turtlebot3_avoidance.launch
