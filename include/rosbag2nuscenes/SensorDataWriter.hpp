#ifndef SENSOR_DATA_WRITER_HPP
#define SENSOR_DATA_WRITER_HPP

#include "MessageTypes.hpp"
#include <filesystem>
#include <queue>
#include <mutex>
#include <thread>
#include <vector>
#include <condition_variable>
#include <pcl/io/pcd_io.h>
#include <opencv2/imgcodecs.hpp>

#define MAX_QUEUE_SIZE 256

namespace fs = std::filesystem;

class SensorDataWriter {
    public:
        SensorDataWriter(int num_workers);

        void writeSensorData(SensorMessageT* msg, fs::path filename);

        void close();
    
    private:

        void writeRadarData(RadarMessageT* msg, fs::path filename);

        void writeLidarData(LidarMessageT* msg, fs::path filename);
        
        void writeCameraData(CameraMessageT* msg, fs::path filename);
        
        void writeFile();
        
        std::queue<std::pair<SensorMessageT*, fs::path>> file_queue_;

        std::vector<std::thread> thread_vector_;

        std::mutex queue_mutex_;

        std::condition_variable queue_ready_;

        bool finished_;

};



#endif