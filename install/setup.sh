#!/bin/bash

BUILD_PX4="true"

echo -e "\e[1;33m Do you want to build PX4 v1.12.3? (y) for simulation (n) if setting this up on on-barod computer \e[0m"
read var
if [ "$var" != "y" ] && [ "$var" != "Y" ]; then
    echo -e "\e[1;33m Skipping PX4 v1.12.3 \e[0m"
    BUILD_PX4="false"
    sleep 1
else
    echo -e "\e[1;33m PX4 v1.12.3 will be built \e[0m"
    BUILD_PX4="true"
    sleep 1
fi

CATKIN_WS=${HOME}/catkin_ws
CATKIN_SRC=${HOME}/catkin_ws/src

if [ ! -d "$CATKIN_WS" ]; then
    echo "Creating $CATKIN_WS ... "
    mkdir -p $CATKIN_SRC
fi

if [ ! -d "$CATKIN_SRC" ]; then
    echo "Creating $CATKIN_SRC ..."
fi

# Configure catkin_Ws
cd $CATKIN_WS
catkin init
catkin config --merge-devel
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

####################################### Setup PX4 v1.10.1 #######################################
if [ "$BUILD_PX4" != "false" ]; then

    #Setting up PX4 PX4-Autopilot
    if [ ! -d "${HOME}/PX4-Autopilot" ]; then
        cd ${HOME}
        git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    else
        echo "PX4-Autopilot already exists. Just pulling latest upstream...."
        cd ${HOME}/PX4-Autopilot
        git pull
    fi
    cd ${HOME}/PX4-Autopilot
    make clean && make distclean
    git checkout v1.12.3 && git submodule init && git submodule update --recursive
    cd ${HOME}/PX4-Autopilot
    bash ./Tools/setup/ubuntu.sh
    DONT_RUN=1 make px4_sitl gazebo

    # Copy PX4 SITL param file
    cp $CATKIN_SRC/px4_fast_planner/config/10017_iris_depth_camera ${HOME}/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/

    source ${HOME}/.bashrc
fi

# Install MAVROS
# 18.04 melodic
# sudo apt install ros-melodic-mavros ros-melodic-mavros-extras -y
# 20.04 noetic
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras -y

####################################### ego-planner-swarm setup #######################################
echo -e "\e[1;33m Adding ego-planner-swarm \e[0m"
# Required for ego-planner-swarm
sudo apt install libarmadillo-dev -y

#Adding ego-planner-swarm
if [ ! -d "$CATKIN_SRC/ego-planner-swarm" ]; then
    echo "Cloning the ego-planner-swarm repo ..."
    cd $CATKIN_SRC
    git clone https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git
    cd ../
else
    echo "ego-planner-swarm already exists. Just pulling ..."
    cd $CATKIN_SRC/ego-planner-swarm
    git pull
    cd ../
fi

####################################### Building catkin_ws #######################################
cd $CATKIN_WS
catkin build multi_map_server
catkin build
