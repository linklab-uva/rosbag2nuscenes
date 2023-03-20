#ifndef SENSOR_DATA_WRITER_HPP
#define SENSOR_DATA_WRITER_HPP

#include "MessageTypes.hpp"
#include <filesystem>
#include <pcl/io/pcd_io.h>

namespace fs = std::filesystem;

class SensorDataWriter {
    public:
        SensorDataWriter();

        void writeRadarData(RadarMessageT msg, fs::path filename);

        void writeLidarData(LidarMessageT msg, fs::path filename);

        void writeCameraData(CameraMessageT msg, fs::path filename);
    
    private:
        pcl::PCDWriter lidar_writer_;
};



#endif