#!/bin/bash
source ~/catkin_ws/devel/setup.bash
source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/PX4-Autopilot
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/px4_fast_planner/models
