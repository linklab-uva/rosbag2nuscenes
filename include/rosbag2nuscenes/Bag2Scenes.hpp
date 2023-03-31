#ifndef BAG2SCENES_HPP
#define BAG2SCENES_HPP

#include <string>
#include <set>
#include <algorithm>
#include <vector>
#include <map>
#include "MessageTypes.hpp"
#include "MessageConverter.hpp"
#include "SensorDataWriter.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rcpputils/asserts.hpp>
#include "yaml-cpp/node/node.h"
#include <nlohmann/json.hpp>
#include "pugixml.hpp"
#include <Eigen/Geometry>
#include <indicators/cursor_control.hpp>
#include <indicators/block_progress_bar.hpp>
#include <indicators/multi_progress.hpp>
#include <thread>
#include <mutex>

namespace fs = std::filesystem;

class Bag2Scenes {
    public:
        Bag2Scenes(const fs::path rosbag_dir, const fs::path param_file);

        void writeScene();

    private:

        std::string generateToken();

        std::vector<float> splitString(std::string str);

        bool is_key_frame(std::string channel, unsigned long timestamp);

        fs::path getFilename(std::string channel, unsigned long timestamp);

        std::string getClosestEgoPose(unsigned long timestamp);
        
        std::string writeLog();

        void writeMap(std::string log_token);

        std::string writeSample();

        void writeSampleData(nlohmann::json& previous_data);

        void writeEgoPose(nlohmann::json& previous_poses);

        void writeCalibratedSensor(std::string frame, std::vector<std::vector<float>> camera_intrinsic);

        std::string writeSensor(std::string channel);


        rosbag2_storage::StorageOptions storage_options_;
        rosbag2_cpp::ConverterOptions converter_options_;
        rosbag2_storage::BagMetadata bag_data_;
        std::string bag_dir_;
        std::vector<std::string> lidar_topics_;
        std::vector<std::string> radar_topics_;
        std::vector<std::string> camera_topics_;
        std::vector<std::string> camera_calibs_;
        std::vector<std::string> topics_of_interest_;
        std::map<std::string, std::string> topic_to_type_;
        std::map<std::string, unsigned long> last_timestamp_received_;
        unsigned long previous_sampled_timestamp_;
        std::unordered_set<std::string> sensors_sampled_;
        int nbr_samples_;
        nlohmann::json samples_;
        std::string previous_sample_token_;
        std::string next_sample_token_;
        std::string current_sample_token_;
        std::string scene_token_;
        std::mutex timestamp_mutex_;
        std::mutex ego_pose_mutex_;
        YAML::Node frame_info_;
        YAML::Node param_yaml_;
        indicators::BlockProgressBar odometry_bar_;
        indicators::BlockProgressBar sensor_data_bar_;
        indicators::MultiProgress<indicators::BlockProgressBar, 2> progress_bars_;
        
};


#endif