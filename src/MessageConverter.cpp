#include "rosbag2nuscenes/MessageConverter.hpp"


MessageConverter::MessageConverter() {

}

RadarMessageT MessageConverter::getRadarMessage() {
    RadarMessageT radar_msg;
    radar_msg.frame_id = radar_msg_.header.frame_id;
    return radar_msg;
}

LidarMessageT MessageConverter::getLidarMessage() {
    LidarMessageT lidar_msg;
    lidar_msg.frame_id = lidar_msg_.header.frame_id;
    return lidar_msg;
}

CameraMessageT MessageConverter::getCameraMessage() {
    CameraMessageT camera_msg;
    return camera_msg;
}

CameraCalibrationT MessageConverter::getCameraCalibration() {
    CameraCalibrationT calibration_msg;
    calibration_msg.frame_id = camera_calib_msg_.header.frame_id;
    for (int i = 0; i < 3; i++) {
        std::vector<float> intrinsic_row;
        for (int j = 0; j < 3; j++) {
            intrinsic_row.push_back(camera_calib_msg_.k[3*i + j]);
        }
        calibration_msg.intrinsic.push_back(intrinsic_row);
    }
    return calibration_msg;
}

OdometryMessageT MessageConverter::getOdometryMessage() {
    OdometryMessageT odometry_msg;
    return odometry_msg;
}

void MessageConverter::getROSMsg(std::string type, std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t> message_wrapper) {
    if (type == "delphi_esr_msgs/msg/EsrTrack") {
        message_wrapper->message = &radar_msg_;
    } else if (type == "sensor_msgs/msg/CompressedImage") {
        message_wrapper->message = &camera_msg_;
    } else if (type == "sensor_msgs/msg/PointCloud2") {
        message_wrapper->message = &lidar_msg_;
    } else if (type == "nav_msgs/msg/Odometry") {
        message_wrapper->message = &odometry_msg_;                                                                                                  
    } else if (type == "sensor_msgs/msg/CameraInfo") {
        message_wrapper->message = &camera_calib_msg_;
    } else {
        printf("Message type unknown, cannot deserialize.\n");
        exit(1);
    }
}