# px4_fast_planner
Integration of [ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git) with [PX4](https://github.com/PX4/Firmware) for planning real-time collision-free and obstacle-free trajectories in bounded environment. This allows you to fly a multi-rotor drone ( equipped with a PX4 autopilot, depth camera, and on-board computer) autonomously while avoiding obstacles.



# Installation
This setup is tested on Ubuntu 20.04 + ROS noetic

* Clone this package into your `~/catkin_ws/src`
```sh
cd ~/catkin_ws/src
git clone https://github.com/rongbohou/px4_fast_planner.git
```

* You can use the `setup.sh` script in the `install` folder to easily setup all dependencies as follows
```sh
cd ~/catkin_ws/src/px4_fast_planner/install
./setup.sh
```
The script will ask if PX4 is to be installled or not. You would likely want to skip this if you are doing the setup on an on-board computer. To do that, just pass `n` to the `setup.sh` script when asked.



# Running in Simulation

* Open a termianl, and run the following command,
```sh
cd ~/catkin_ws
source ./src/px4_fast_planner/setup_env.sh
roslaunch px4_fast_planner px4_ego_planner.launch
```


**NOTE: It may take some time to download some Gazebo world models in the first time you run the simulation**

* To command the drone to fly to a target pose, publish a single message to the `/move_base_simple/goal` topic as follows
```sh
rostopic pub --once /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 19.0
    y: 15.0
    z: 3.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```
