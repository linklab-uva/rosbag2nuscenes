#ifndef MESSAGE_CONVERTER_HPP
#define MESSAGE_CONVERTER_HPP

#include "MessageTypes.hpp"
#include "delphi_esr_msgs/msg/esr_track.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"



class MessageConverter {
    public:
        MessageConverter();

        RadarMessageT convertRadarMessage(delphi_esr_msgs::msg::EsrTrack msg);
        
        LidarMessageT convertLidarMessage(sensor_msgs::msg::PointCloud2 msg);

        CameraMessageT convertCameraMessage(sensor_msgs::msg::CompressedImage msg);


};


#endif