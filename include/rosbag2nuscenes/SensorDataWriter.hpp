#ifndef SENSOR_DATA_WRITER_HPP
#define SENSOR_DATA_WRITER_HPP

#define PCL_NO_PRECOMPILE

#include "MessageTypes.hpp"
#include <filesystem>
#include <boost/circular_buffer.hpp>
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

        void writeRadarData(RadarMessageT msg, fs::path filename);

        void writeLidarData(LidarMessageT msg, fs::path filename);
        
        void writeCameraData(CameraMessageT msg, fs::path filename);
        
        void writeFile();
        
        boost::circular_buffer<std::pair<SensorMessageT*, fs::path>> file_queue_;

        std::vector<std::thread> thread_vector_;

        std::mutex queue_mutex_;

        std::condition_variable queue_full_;

        std::condition_variable queue_empty_;

        bool finished_;

};



#endif