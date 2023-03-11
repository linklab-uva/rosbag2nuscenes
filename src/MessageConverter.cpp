#include "rosbag2nuscenes/MessageConverter.hpp"


MessageConverter::MessageConverter() {

}

RadarMessageT MessageConverter::convertRadarMessage(delphi_esr_msgs::msg::EsrTrack msg) {
    RadarMessageT radar_msg;
    return radar_msg;
}

LidarMessageT MessageConverter::convertLidarMessage(sensor_msgs::msg::PointCloud2 msg) {
    LidarMessageT lidar_msg;
    return lidar_msg;
}

CameraMessageT MessageConverter::convertCameraMessage(sensor_msgs::msg::CompressedImage msg) {
    CameraMessageT camera_msg;
    return camera_msg;
}