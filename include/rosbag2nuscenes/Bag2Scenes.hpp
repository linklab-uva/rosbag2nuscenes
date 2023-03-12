#ifndef BAG2SCENES_HPP
#define BAG2SCENES_HPP

#include <string>
#include <vector>
#include <map>
#include "MessageTypes.hpp"
#include "MessageConverter.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <chrono>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rcpputils/asserts.hpp>
#include "yaml-cpp/node/node.h"
#include <nlohmann/json.hpp>
#include <Eigen/Geometry>

namespace fs = std::filesystem;

class Bag2Scenes {
    public:
        Bag2Scenes(const fs::path rosbag_dir, const fs::path param_file);

        void writeScene();

    private:

        static std::string generateToken() {
            char token[17];
            for (int i = 0; i < 17; i++) {
                sprintf(token + i, "%x", rand() % 16);
            }
            return std::string(token);
        }
        
        /**
         * @brief 
         * 
         * @return std::string Token of log
         */
        std::string writeLog();

        void writeMap(std::string log_token);

        /**
         * @brief 
         * 
         * @return std::string Token of sample
         */
        std::string writeSample();

        /**
         * @brief 
         * 
         * @param data 
         * @return std::string Token of sample data if is_key_frame, else 0
         */
        std::string writeSampleData(SensorMessageT data);

        void writeEgoPose(Eigen::Quaternionf ego_pose);

        void writeCalibratedSensor(Eigen::Quaternionf sensor_pose);

        std::string writeSensor(std::string channel, std::string modality);


        rosbag2_cpp::readers::SequentialReader reader_;
        rosbag2_storage::BagMetadata bag_data_;
        std::string bag_dir_;
        std::vector<std::string> lidar_topics_;
        std::vector<std::string> radar_topics_;
        std::vector<std::string> camera_topics_;
        std::vector<std::string> topics_of_interest_;
        YAML::Node topic_info_;
        YAML::Node param_yaml_;

};


#endif