# rosbag2nuscenes

This codebase converts ROS2 bag files to the nuScenes dataset format. The conversion library requires a valid ROS2 bag and parameter file that specifies which topics to convert, information of the scene in the bag file, and a path to a URDF of the ego vehicle. An example parameter file is provided [here](params/mit.yaml).

## Build

### Requirements
* A sourced ROS2 installation
* A compiler that supports C++17
* OpenCV
* PCL
* Eigen3

The library currently has delphi_esr_msgs as a dependency, found [here](https://github.com/astuff/astuff_sensor_msgs). This is listed as a dependency because the radar messages used for developing the library were of type `delphi_esr_msgs/msg/EsrTrack`. If you are using a different radar message type, this dependency can be removed with changes to `src/MessageConverter.cpp:convertRadarMessage()`.

```
git clone --recurse-submodules https://github.com/linklab-uva/rosbag2nuscenes.git && cd rosbag2nuscenes/
mkdir build/ && cd build/
cmake ..
make
./rosbag2nuscenes /path/to/bag/file /path/to/parameter/file
```

