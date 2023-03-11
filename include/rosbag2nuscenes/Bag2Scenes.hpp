#ifndef BAG2SCENES_HPP
#define BAG2SCENES_HPP

#include <string>
#include <vector>
#include "MessageTypes.hpp"
#include <filesystem>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <Eigen/Geometry>

class Bag2Scenes {
    public:
        Bag2Scenes(const std::filesystem::path rosbag_dir, const std::filesystem::path param_file);

        void writeScene();

    private:
        /**
         * @brief 
         * 
         * @return unsigned long Token of log
         */
        unsigned long writeLog();

        void writeMap(unsigned long log_token);

        /**
         * @brief 
         * 
         * @return unsigned long Token of sample
         */
        unsigned long writeSample();

        /**
         * @brief 
         * 
         * @param data 
         * @return unsigned long Token of sample data if is_key_frame, else 0
         */
        unsigned long writeSampleData(SensorMessageT data);

        void writeEgoPose(Eigen::Quaternionf ego_pose);

        void writeCalibratedSensor(Eigen::Quaternionf sensor_pose);

        
        unsigned long writeSensor(std::string channel, std::string modality);

        rosbag2_cpp::readers::SequentialReader reader;


};


#endif