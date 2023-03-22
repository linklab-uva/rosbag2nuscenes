#ifndef SENSOR_DATA_WRITER_HPP
#define SENSOR_DATA_WRITER_HPP

#include "MessageTypes.hpp"
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <opencv2/imgcodecs.hpp>

namespace fs = std::filesystem;

class SensorDataWriter {
    public:
        SensorDataWriter();

        void writeRadarData(RadarMessageT msg, fs::path filename);

        void writeLidarData(LidarMessageT msg, fs::path filename);

        void writeCameraData(CameraMessageT msg, fs::path filename);
    
    private:
};



#endif