#include "rosbag2nuscenes/Bag2Scenes.hpp"

Bag2Scenes::Bag2Scenes(const fs::path rosbag_dir, const fs::path param_file) {
    // Read Parameter File
    try {
        param_yaml_ = YAML::LoadFile(param_file);
    } catch (std::exception e) {
        printf("Error reading %s\n", param_file.c_str());
        exit(1);
    }
    // Storage Options
    rosbag2_storage::StorageOptions storage_options{};
    storage_options.uri = rosbag_dir;
    storage_options.storage_id = "sqlite3";
    // Converter Options
    rosbag2_cpp::ConverterOptions converter_options{};
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";
    // Open Bag and Store Metadata
    try {
        reader_.open(storage_options, converter_options);
    } catch (std::exception e) {
        exit(1);
    }
    std::vector<rosbag2_storage::TopicMetadata> topic_types = reader_.get_all_topics_and_types();
    bag_data_ = reader_.get_metadata();
    std::map<std::string, std::string> topic_to_type;
    for (std::vector<rosbag2_storage::TopicMetadata>::iterator itr  = topic_types.begin(); itr != topic_types.end(); itr++) {
        topic_to_type.insert({itr->name, itr->type});
    }
    for (YAML::Node::iterator itr = param_yaml_["SENSOR_INFO"].begin(); itr != param_yaml_["SENSOR_INFO"].end(); itr++) {
        std::string sensor_name = itr->first.as<std::string>();
        std::string modality = sensor_name.substr(0, sensor_name.find("_"));
        std::string topic = itr->second["TOPIC"].as<std::string>();
        if (topic != "null") {
            topic_info_[topic]["TYPE"] = topic_to_type[topic];
            topic_info_[topic]["SENSOR"] = sensor_name;
            topics_of_interest_.push_back(topic);
            if (modality == "LIDAR") {
                lidar_topics_.push_back(topic);
            } else if (modality == "RADAR") {
                radar_topics_.push_back(topic);
            } else if (modality == "CAMERA") {
                camera_topics_.push_back(topic);
            } else {
                printf("Invalid sensor %s in %s. Ensure sensor is of type LIDAR, RADAR, or CAMERA and is named [SENSOR TYPE]_[SENSOR LOCATION]", sensor_name.c_str(), param_file.c_str());
            }
        }
    }
    topics_of_interest_.push_back(param_yaml_["BAG_INFO"]["ODOM_TOPIC"].as<std::string>());
    // Storage Filter
    rosbag2_storage::StorageFilter storage_filter{};
    storage_filter.topics = topics_of_interest_;
    reader_.set_filter(storage_filter);
    bag_dir_ = rosbag_dir.parent_path().filename();
    srand(time(0));
}

void Bag2Scenes::writeScene() {
    int total_msgs = 0;
    for (std::vector<rosbag2_storage::TopicInformation>::iterator itr = bag_data_.topics_with_message_count.begin(); itr != bag_data_.topics_with_message_count.end(); itr++) {
        if (std::find(topics_of_interest_.begin(), topics_of_interest_.end(), itr->topic_metadata.name) != topics_of_interest_.end()) {
            total_msgs += itr->message_count;
        }
    }
    if (!fs::exists("v1.0-mini")) {
        fs::create_directories("v1.0-mini/samples");
        fs::create_directory("v1.0-mini/sweeps");
    }
    std::string log_token = writeLog();
    writeMap(log_token);
}

std::string Bag2Scenes::writeLog() {
    std::string log_token = generateToken();
    nlohmann::json logs;
    if (fs::exists("v1.0-mini/log.json")) {
        std::ifstream log_in("v1.0-mini/log.json");
        logs = nlohmann::json::parse(log_in);
        log_in.close();
    }
    nlohmann::json new_log;
    new_log["token"] = log_token;
    new_log["logfile"] = bag_dir_;
    new_log["vehicle"] = param_yaml_["BAG_INFO"]["TEAM"].as<std::string>();
    std::stringstream ss;
    const long time = (const long) std::floor(bag_data_.starting_time.time_since_epoch().count() * std::pow(10,-9));
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d");
    new_log["date_captured"] = ss.str();
    new_log["location"] = param_yaml_["BAG_INFO"]["TRACK"].as<std::string>();
    logs.push_back(new_log);
    std::ofstream log_out("v1.0-mini/log.json");
    log_out << std::setw(4) << logs << std::endl;
    return log_token;
}

void Bag2Scenes::writeMap(std::string log_token) {
    nlohmann::json maps;
    if (fs::exists("v1.0-mini/map.json")) {
        std::ifstream map_in("v1.0-mini/map.json");
        maps = nlohmann::json::parse(map_in);
        map_in.close();
        for (nlohmann::json& map : maps) {
            if (map["category"] == param_yaml_["BAG_INFO"]["TRACK"].as<std::string>()) {
                map["log_tokens"].push_back(log_token);
                std::ofstream map_out("v1.0-mini/map.json");
                map_out << std::setw(4) << maps << std::endl;
                map_out.close();
                return;
            }
        }
    }
    nlohmann::json new_map;
    new_map["token"] = generateToken();
    new_map["log_tokens"] = std::vector<std::string>{log_token};
    new_map["category"] = param_yaml_["BAG_INFO"]["TRACK"].as<std::string>();
    new_map["filename"] = "";
    maps.push_back(new_map);
    std::ofstream map_out("v1.0-mini/map.json");
    map_out << std::setw(4) << maps << std::endl;
    map_out.close();
    return;
}

std::string Bag2Scenes::writeSample() {
    std::string temp = 0;
    return temp;
}

std::string Bag2Scenes::writeSampleData(SensorMessageT data) {
    std::string temp = 0;
    return temp;
}

void Bag2Scenes::writeEgoPose(Eigen::Quaternionf ego_pose) {

}

void Bag2Scenes::writeCalibratedSensor(Eigen::Quaternionf sensor_pose) {

}

std::string Bag2Scenes::writeSensor(std::string channel, std::string modality) {
    std::string temp = 0;
    return temp;
}