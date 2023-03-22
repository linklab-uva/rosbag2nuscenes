#ifndef MESSAGE_CONVERTER_HPP
#define MESSAGE_CONVERTER_HPP

#include <cstring>
#include "MessageTypes.hpp"
#include "delphi_esr_msgs/msg/esr_track.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <cv_bridge/cv_bridge.h>



class MessageConverter {
    public:
        MessageConverter();

        RadarMessageT getRadarMessage();
        
        LidarMessageT getLidarMessage();

        CameraMessageT getCameraMessage();

        CameraCalibrationT getCameraCalibration();

        OdometryMessageT getOdometryMessage();

        void getROSMsg(std::string type, std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t> message_wrapper);

    private:
        delphi_esr_msgs::msg::EsrTrack radar_ros_msg_;
        sensor_msgs::msg::PointCloud2 lidar_ros_msg_;
        sensor_msgs::msg::CompressedImage camera_ros_msg_;
        sensor_msgs::msg::CameraInfo camera_info_ros_msg_;
        nav_msgs::msg::Odometry odometry_ros_msg_;



};


#endif