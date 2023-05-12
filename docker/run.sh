#!/bin/bash

docker run --rm -it --mount type=bind,src=$1,target=$1 --mount type=bind,src=$3,target=$3 rosbag2nuscenes $@
