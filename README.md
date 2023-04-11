# rosbag2nuscenes

This codebase converts ROS2 bag files to the nuScenes dataset format. The conversion library requires a valid ROS2 bag and parameter file that specifies which topics to convert, information of the scene in the bag file, and a path to a URDF of the ego vehicle. An example parameter file is provided [here](params/mit.yaml).

## Build

### Requirements
* A sourced ROS2 installation
* A compiler that supports C++17
* OpenCV
* PCL
* Eigen3
* yaml-cpp
* pcl-ros

The library currently has delphi_esr_msgs as a dependency, found [here](https://github.com/astuff/astuff_sensor_msgs). This is listed as a dependency because the radar messages used for developing the library were of type `delphi_esr_msgs/msg/EsrTrack`. If you are using a different radar message type, this dependency can be removed with changes to `src/MessageConverter.cpp:convertRadarMessage()`.

```
git clone --recurse-submodules https://github.com/linklab-uva/rosbag2nuscenes.git && cd rosbag2nuscenes/
mkdir build/ && cd build/
cmake ..
make
./rosbag2nuscenes /path/to/bag/file /path/to/parameter/file
```

## nuScenes Dev-Kit

Along with their dataset, the Motional team also released a python development kit to interact with and explore the dataset. This development kit is mostly  functional with datasets generated using the conversion library, but there are some hard-coded sensor names that cause issues when sensors are named differently than the sensors in the nuScenes dataset. We have created a fork of the nuScenes dev-kit that is compatible with datasets created with our conversion tool, which should have been cloned with the conversion tool. Follow the instructions [here](https://github.com/linklab-uva/nuscenes-devkit/blob/master/docs/installation.md) to build the dev-kit from source.

The Jupyter notebook tutorial that accompanies the dev-kit is also available in this repo. Once you have created your dataset, you can use this notebook as a starting point to get comfortable exploring your data.

Note: There is no sadly no way of annotating all of the objects in your bag file automatically, so the conversion library creates dummy entries in all json files associated with annotations in the nuScenes dataset in order to maintain dompatibility with the dev-kit.
