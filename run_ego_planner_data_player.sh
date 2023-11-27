#!/bin/bash

# check input arguments
if [ "$#" -lt 1 ]; then
    echo "Usage: $(basename $0) data.bag"
    exit 1
fi

rosbag_file="$1"
echo "rosbag_file: " ${rosbag_file}

sleep 2s

source ./setup_env.sh

# launch px4_ego_planner_data_player.launch
gnome-terminal --window -e "roslaunch px4_fast_planner px4_ego_planner_data_player.launch"

sleep 8s

# rosbag play
gnome-terminal --window -e "rosbag play ${rosbag_file} --topics /mavros/local_position/odom /camera/pose /camera/depth/image_raw /tf /mavros/local_position/pose /move_base_simple/goal"

sleep 2s
