#include "rosbag2nuscenes/Bag2Scenes.hpp"

Bag2Scenes::Bag2Scenes(const std::filesystem::path rosbag_dir, const std::filesystem::path param_file) {
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
    std::map<std::string, std::string> topic_to_type;
    std::vector<std::string> topics_of_interest;
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
            topics_of_interest.push_back(topic);
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
    // Storage Filter
    rosbag2_storage::StorageFilter storage_filter{};
    storage_filter.topics = topics_of_interest;
    reader_.set_filter(storage_filter);
}

void Bag2Scenes::writeScene() {
    rosbag2_storage::BagMetadata bag_data = reader_.get_metadata();

}

unsigned long Bag2Scenes::writeLog() {
    unsigned long temp = 0;
    return temp;
}

void Bag2Scenes::writeMap(unsigned long log_token) {

}

unsigned long Bag2Scenes::writeSample() {
    unsigned long temp = 0;
    return temp;
}

unsigned long Bag2Scenes::writeSampleData(SensorMessageT data) {
    unsigned long temp = 0;
    return temp;
}

void Bag2Scenes::writeEgoPose(Eigen::Quaternionf ego_pose) {

}

void Bag2Scenes::writeCalibratedSensor(Eigen::Quaternionf sensor_pose) {

}

unsigned long Bag2Scenes::writeSensor(std::string channel, std::string modality) {
    unsigned long temp = 0;
    return temp;
}