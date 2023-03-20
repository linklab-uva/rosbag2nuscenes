#ifndef MESSAGE_TYPES_HPP
#define MESSAGE_TYPES_HPP

#include <string>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct SensorMessageT {
    unsigned long timestamp;
    std::string frame_id;
};

struct RadarMessageT : SensorMessageT {
    //TODO
};

struct LidarMessageT : SensorMessageT {
    pcl::PointCloud<pcl::PointXYZI> cloud;
};

struct CameraMessageT : SensorMessageT {
    //TODO
};

struct OdometryMessageT {
    unsigned long timestamp;
    std::vector<double> position;
    std::vector<double> orientation;
};

struct CameraCalibrationT {
    std::string frame_id;
    std::vector<std::vector<float>> intrinsic;
};


#endif