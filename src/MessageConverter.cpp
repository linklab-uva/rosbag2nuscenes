#include "rosbag2nuscenes/MessageConverter.hpp"


MessageConverter::MessageConverter() {

}

RadarMessageT MessageConverter::getRadarMessage() {
    RadarMessageT radar_msg;
    radar_msg.frame_id = radar_ros_msg_.header.frame_id;
    return radar_msg;
}

LidarMessageT MessageConverter::getLidarMessage() {
    LidarMessageT lidar_msg;
    lidar_msg.frame_id = lidar_ros_msg_.header.frame_id;
    return lidar_msg;
}

CameraMessageT MessageConverter::getCameraMessage() {
    CameraMessageT camera_msg;
    return camera_msg;
}

CameraCalibrationT MessageConverter::getCameraCalibration() {
    CameraCalibrationT calibration_msg;
    calibration_msg.frame_id = camera_info_ros_msg_.header.frame_id;
    for (int i = 0; i < 3; i++) {
        std::vector<float> intrinsic_row;
        for (int j = 0; j < 3; j++) {
            intrinsic_row.push_back(camera_info_ros_msg_.k[3*i + j]);
        }
        calibration_msg.intrinsic.push_back(intrinsic_row);
    }
    return calibration_msg;
}

OdometryMessageT MessageConverter::getOdometryMessage() {
    OdometryMessageT odometry_msg;
    std::stringstream ss;
    ss << odometry_ros_msg_.header.stamp.sec << "." << odometry_ros_msg_.header.stamp.nanosec;
    odometry_msg.timestamp = stoi(ss.str());
    odometry_msg.position = { odometry_ros_msg_.pose.pose.position.x, odometry_ros_msg_.pose.pose.position.y, odometry_ros_msg_.pose.pose.position.z };
    odometry_msg.orientation = { odometry_ros_msg_.pose.pose.orientation.w, odometry_ros_msg_.pose.pose.orientation.x, odometry_ros_msg_.pose.pose.orientation.y, odometry_ros_msg_.pose.pose.orientation.z };
    return odometry_msg;
}

void MessageConverter::getROSMsg(std::string type, std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t> message_wrapper) {
    if (type == "delphi_esr_msgs/msg/EsrTrack") {
        message_wrapper->message = &radar_ros_msg_;
    } else if (type == "sensor_msgs/msg/CompressedImage") {
        message_wrapper->message = &camera_ros_msg_;
    } else if (type == "sensor_msgs/msg/PointCloud2") {
        message_wrapper->message = &lidar_ros_msg_;
    } else if (type == "nav_msgs/msg/Odometry") {
        message_wrapper->message = &odometry_ros_msg_;                                                                                                  
    } else if (type == "sensor_msgs/msg/CameraInfo") {
        message_wrapper->message = &camera_info_ros_msg_;
    } else {
        printf("Message type unknown, cannot deserialize.\n");
        exit(1);
    }
}