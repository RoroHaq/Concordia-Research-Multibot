# Multi Robot Navigation (Full Research PDF included)
## Multiple Turtlebot3 robot support in Gazebo
The ROS2 project  scalable solution for launching multiple TurtleBot3 robots with navigation capabilities using the Navigation2 (Nav2) stack. By leveraging namespaces in ROS2, this project enables the seamless deployment of multiple TurtleBot3 robots in a simple and organized manner. Each robot instance can be differentiated by its unique namespace, ensuring independence and preventing naming conflicts.

The 'main' branch includes an implementation that functions with the humble framework

```
mkdir -p robot_ws/src
cd robot_ws/src

# cloning via https
git clone https://github.com/RoroHaq/Concordia-Research-Multibot.git

# cloning via ssh
git clone  git@github.com:RoroHaq/Concordia-Research-Multibot.git

cd robot_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -r -y
```

## Setup Models
In order to properly run the world, you will need to copy the models in your .gazebo/models folder. Luckily I made a script for that for ease of use with the following command in the project root folder

```
source setup.bash
```

this is so that it can read the following found in sdf/world files:

```
<uri>model://path/to/model</uri>
```
## Run without nav2 stack
```
ros2 launch turtlebot3_multi_robot gazebo_multi_world.launch.py 
```
# turtlebot3_multi_robot

![image](https://github.com/arshadlab/turtlebot3_multi_robot/assets/85929438/fc958709-018d-48d2-b5b6-6674b53913c8)

## Run with nav2 stack

#### Robot Configuration

The arrangement of robots is configured in gazebo_multi_nav2_world.launch.py launch file. A potential future enhancement could involve retrieving the configurations from a file, such as json.

Names and poses for the robots in nav2 example
```
 robots = [
 {'name': 'tb1', 'x_pose': '-1.5', 'y_pose': '-0.5', 'z_pose': 0.01},
 {'name': 'tb2', 'x_pose': '-1.5', 'y_pose': '0.5', 'z_pose': 0.01},
 {'name': 'tb3', 'x_pose': '1.5', 'y_pose': '-0.5', 'z_pose': 0.01},
 {'name': 'tb4', 'x_pose': '1.5', 'y_pose': '0.5', 'z_pose': 0.01},
 # …
 # …
 ]
```

There are 2 ways to run this project, via manually making 3 separate terminals, or running the bash script I have created.

## Script Version

1. Go to the root of the repo, where the bash files are.
2. Afterwards you run the following command, depending on if you want the EV9 or the Empty world.

```
# for EV9

bash run_multi_nav_EV9.bash

# For Empty World

bash run_multi_nav_empty_world.bash
```

And then the project will be running
### Main Terminal

First cd into turtlebot3_multi_robot
```
cd src/turtlebot3_multi_robot
```
Build the Files:
```
source /opt/ros/humble/setup.bash
colcon build
colcon test
source install/setup.bash
```
Running the main launch file:
```
ros2 launch turtlebot3_multi_robot gazebo_multi_nav2_world.launch.py 
```

### 2nd Terminal
Running the Formation creation script:
```
#Follow the Build Files step forom Main terminal
ros2 run turtlebot3_multi_robot goal_listener_and_saver.py 
```

### 3rd Terminal
Running the script for reserving the area:
```
#Follow the Build FIles step forom Main terminal
ros2 run turtlebot3_multi_robot spawn.py 
```
Testing the functionality in diiferent scenarios:

https://github.com/user-attachments/assets/9560e6d0-295c-47f9-a194-e045a8d12125


https://github.com/user-attachments/assets/c4b97bc5-324b-43f7-bdde-c9dacbe10545



https://github.com/user-attachments/assets/83071ee5-5f14-4a5a-99f0-39ac3d3b996b




**References**: https://medium.com/@arshad.mehmood/a-guide-to-multi-robot-navigation-utilizing-turtlebot3-and-nav2-cd24f96d19c6


