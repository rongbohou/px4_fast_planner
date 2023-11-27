#!/bin/bash

# input:  /mavros/local_position/odom /camera/pose /camera/depth/image_raw

# output: /planning/ref_traj /mavros/setpoint_raw/local

# inner:  /iris_0_ego_planner_node/goal_point /iris_0_ego_planner_node/global_list
#         /iris_0_ego_planner_node/optimal_list /iris_0_ego_planner_node/a_star_list
#         /iris_0_ego_planner_node/init_list /planning/position_cmd_vis
#         /iris_0_ego_planner_node/grid_map/occupancy_inflate

# others: /mavros/state /camera/rgb/image_raw /mavros/local_position/pose /tf

# Attention: occupancy_inflate rgb/image_raw depth/image_raw are big

bag_dir=~/drone/dataset/planning/

cd $bag_dir

rosbag record /mavros/local_position/odom /camera/pose /camera/depth/image_raw \
    /planning/ref_traj /mavros/setpoint_raw/local \
    /iris_0_ego_planner_node/goal_point /iris_0_ego_planner_node/global_list \
    /iris_0_ego_planner_node/optimal_list /iris_0_ego_planner_node/a_star_list \
    /iris_0_ego_planner_node/init_list /planning/position_cmd_vis \
    /iris_0_ego_planner_node/grid_map/occupancy_inflate \
    /mavros/state /camera/rgb/image_raw /mavros/local_position/pose /tf

cd -

pwd
