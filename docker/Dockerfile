FROM osrf/ros:humble-desktop

RUN apt update && \
    apt install -y --no-install-recommends \
    nlohmann-json3-dev \
    ros-humble-pcl-ros \
    locales \
    software-properties-common \
    git

ENV TZ=America/New_York
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN mkdir -p /colcon_ws/src

WORKDIR /colcon_ws/src

RUN git clone https://github.com/astuff/astuff_sensor_msgs -b dashing-devel

WORKDIR /colcon_ws

RUN ls /opt/ros/humble

RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select delphi_esr_msgs


COPY . /rosbag2nuscenes


RUN mkdir /rosbag2nuscenes/build

WORKDIR /rosbag2nuscenes/build

RUN . /opt/ros/humble/setup.sh && \
    . /colcon_ws/install/setup.sh && \
    cmake .. && \
    make

WORKDIR /rosbag2nuscenes/docker

ENTRYPOINT ["./entrypoint.sh"]
