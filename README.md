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
   git clone https://github.com/MauBenavidesS/TurtleBot3.git

## Build
git clone https://github.com/MauBenavidesS/TurtleBot3.git
2. cd ~/TurtleBot3
3. sudo ./setup_turtlebot3.sh

## Execute TurtleBot3
- roslaunch turtlebot3_gazebo turtlebot3_world.launch
- roslaunch turtlebot3_avoidance turtlebot3_avoidance.launch
