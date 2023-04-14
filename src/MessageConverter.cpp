#include "rosbag2nuscenes/MessageConverter.hpp"


MessageConverter::MessageConverter() {}

RadarMessageT MessageConverter::getRadarMessage() {
    std::stringstream ss;
    ss << radar_ros_msg_.header.stamp.sec << radar_ros_msg_.header.stamp.nanosec;
    RadarPointT radar_point { (float) (radar_ros_msg_.track_range * cos(radar_ros_msg_.track_angle * 3.14159/180)),
                    (float) (radar_ros_msg_.track_range * sin(radar_ros_msg_.track_angle * 3.14159/180)),
                    0.0,
                    4,
                    radar_ros_msg_.track_id,
                    radar_ros_msg_.track_width,
                    radar_ros_msg_.track_range_rate,
                    radar_ros_msg_.track_lat_rate,
                    radar_ros_msg_.track_range_rate,
                    radar_ros_msg_.track_lat_rate,
                    1,
                    3,
                    0, 0, 0, 1, 0, 0} ;
    RadarMessageT radar_msg;
    radar_msg.timestamp = stoul(ss.str());
    radar_msg.frame_id = radar_ros_msg_.header.frame_id;
    radar_msg.points = std::vector<RadarPointT> {radar_point};
    return radar_msg;
}

LidarMessageT MessageConverter::getLidarMessage() {
    LidarMessageT lidar_msg;
    lidar_msg.frame_id = lidar_ros_msg_.header.frame_id;
    std::stringstream ss;
    ss << lidar_ros_msg_.header.stamp.sec << lidar_ros_msg_.header.stamp.nanosec;
    lidar_msg.timestamp = stoul(ss.str());
    pcl::fromROSMsg(lidar_ros_msg_, lidar_msg.cloud);
    return lidar_msg;
}

CameraMessageT MessageConverter::getCameraMessage() {
    CameraMessageT camera_msg;
    camera_msg.frame_id = camera_ros_msg_.header.frame_id;
    std::stringstream ss;
    ss << camera_ros_msg_.header.stamp.sec << camera_ros_msg_.header.stamp.nanosec;
    camera_msg.timestamp = stoul(ss.str());
    camera_msg.image = cv_bridge::toCvCopy(camera_ros_msg_, "rgb8")->image;
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
    ss << odometry_ros_msg_.header.stamp.sec << odometry_ros_msg_.header.stamp.nanosec;
    odometry_msg.timestamp = stoul(ss.str());
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