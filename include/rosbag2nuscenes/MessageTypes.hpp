#ifndef MESSAGE_TYPES_HPP
#define MESSAGE_TYPES_HPP

#include <string>
#include <vector>

struct SensorMessageT {
    unsigned long timestamp;
    std::string frame_id;
};

struct RadarMessageT : SensorMessageT {
    //TODO
};

struct LidarMessageT : SensorMessageT {
    //TODO
};

struct CameraMessageT : SensorMessageT {
    //TODO
};

struct OdometryMessageT {
    unsigned long timestamp;
};

struct CameraCalibrationT {
    std::string frame_id;
    std::vector<std::vector<float>> intrinsic;
};


#endif