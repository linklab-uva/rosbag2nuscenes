#ifndef MESSAGE_CONVERTER_HPP
#define MESSAGE_CONVERTER_HPP

#include <cstring>
#include "MessageTypes.hpp"
#include "delphi_esr_msgs/msg/esr_track.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>



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
        delphi_esr_msgs::msg::EsrTrack radar_msg_;
        sensor_msgs::msg::PointCloud2 lidar_msg_;
        sensor_msgs::msg::CompressedImage camera_msg_;
        sensor_msgs::msg::CameraInfo camera_calib_msg_;
        nav_msgs::msg::Odometry odometry_msg_;



};


#endif