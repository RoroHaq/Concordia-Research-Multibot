#!/bin/bash

# This script will be used for the basic setup thats needed, like moving the initial models around for 
# the environment to work properly in your Humble/classic system.
path="$HOME/.gazebo/models"
TITLE1="my_term_1_$$"

echo "Starting model movement..."

cd src/turtlebot3_multi_robot

echo "Retrieving models..."

cp -r models/* $path

echo "Model transfer done!"

cleanup() {
    echo "Cleaning up..."
    pkill -f "$TITLE1"
    echo "Script exiting..."
    exit 0
}

trap cleanup SIGINT

# Running the project
gnome-terminal -- bash -c "
        export PROMPT_COMMAND='';
        echo -ne '\033]0;$TITLE1\007'; 
        colcon build 2>&1 | sed 's/\x1b\].*[\aG]//g'; 
        source install/setup.bash; ros2 launch turtlebot3_multi_robot gazebo_mapping.launch.py; 
        exec bash" &


cd ../../

while true; do sleep 1; done
