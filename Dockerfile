FROM osrf/ros:humble-desktop

RUN apt update && \
    apt install --no-install-recommends \
    nlohmann-json3-dev \
    ros-humble-pcl-conversions

COPY . /rosbag2nuscenes

WORKDIR /rosbag2nuscenes

RUN mkdir build

WORKDIR /rosbag2nuscenes/build

RUN cmake ..

RUN make