#!/bin/bash

source /opt/ros/humble/setup.bash
source /colcon_ws/install/setup.bash
cd ../build/
./rosbag2nuscenes $@
