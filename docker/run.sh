#!/bin/bash

docker run --rm -it --mount type=bind,src=$1,target=$1 rosbag2nuscenes $@
