#!/bin/bash

# This script will be used for the basic setup thats needed, like moving the initial models around for 
# the environment to work properly in your Humble/classic system.
path="$HOME/.gazebo/models"

echo "Starting model movement..."

cd src/turtlebot3_multi_robot

echo "Retrieving models..."

cp -r models/* $path

echo "Model transfer done!"


# Running the project
gnome-terminal -- bash -c "colcon build; source install/setup.bash; ros2 launch turtlebot3_multi_robot gazebo_mapping.launch.py; exec bash"

cd ../../