#!/bin/bash
set -e

echo "potential-vision start"

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# setup env
export MVCAM_SDK_PATH=/opt/MVS
export MVCAM_SDK_VERSION=
export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export MVCAM_GENICAM_CLPROTOCOL=/opt/MVS/lib/CLProtocol
export ALLUSERSPROFILE=/opt/MVS/MVFG

export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/MVS/lib/32:$LD_LIBRARY_PATH

export ROS_DOMAIN_ID=7

# source ros
source /opt/ros/humble/setup.bash

# build workspace
colcon build --symlink-install

# source workspace
source install/setup.bash

# run node
ros2 launch vision vision.launch.py
