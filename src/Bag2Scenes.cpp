#include "rosbag2nuscenes/Bag2Scenes.hpp"
#include <iostream>

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
    for (std::vector<rosbag2_storage::TopicMetadata>::iterator itr  = topic_types.begin(); itr != topic_types.end(); itr++) {
        topic_to_type_.insert({itr->name, itr->type});
    }
    for (YAML::Node::iterator itr = param_yaml_["SENSOR_INFO"].begin(); itr != param_yaml_["SENSOR_INFO"].end(); itr++) {
        std::string sensor_name = itr->first.as<std::string>();
        std::string modality = sensor_name.substr(0, sensor_name.find("_"));
        std::string topic = itr->second["TOPIC"].as<std::string>();
        frame_info_[itr->second["FRAME"].as<std::string>()]["name"] = sensor_name;
        if (topic != "null") {
            topics_of_interest_.push_back(topic);
            if (modality == "LIDAR") {
                lidar_topics_.push_back(topic);
            } else if (modality == "RADAR") {
                radar_topics_.push_back(topic);
            } else if (modality == "CAMERA") {
                camera_topics_.push_back(topic);
                camera_calibs_.push_back(itr->second["CALIB"].as<std::string>());
                topics_of_interest_.push_back(itr->second["CALIB"].as<std::string>());
            } else {
                printf("Invalid sensor %s in %s. Ensure sensor is of type LIDAR, RADAR, or CAMERA and is named [SENSOR TYPE]_[SENSOR LOCATION]", sensor_name.c_str(), param_file.c_str());
                exit(1);
            }
        }
    }
    topics_of_interest_.push_back(param_yaml_["BAG_INFO"]["ODOM_TOPIC"].as<std::string>());
    // Storage Filter
    rosbag2_storage::StorageFilter storage_filter{};
    storage_filter.topics = topics_of_interest_;
    reader_.set_filter(storage_filter);
    // Deserialization Tools
    rosbag2_cpp::SerializationFormatConverterFactory factory;
    cdr_deserializer_ = factory.load_deserializer("cdr");
    bag_dir_ = rosbag_dir.parent_path().filename();
    srand(time(0));
}

void Bag2Scenes::writeScene() {
    MessageConverter message_converter;
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
    writeLog();
    std::unordered_set<std::string> calibrated_sensors;
    for (int i = 0; i < total_msgs; i++) {
        if (reader_.has_next()) {
            auto serialized_message = reader_.read_next();
            auto message_wrapper = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
            std::string msg_type = topic_to_type_[serialized_message->topic_name];
            message_converter.getROSMsg(msg_type, message_wrapper);
            std::cout << msg_type << std::endl;
            auto library = rosbag2_cpp::get_typesupport_library(msg_type, "rosidl_typesupport_cpp");
            auto type_support = rosbag2_cpp::get_typesupport_handle(msg_type, "rosidl_typesupport_cpp", library);
            cdr_deserializer_->deserialize(serialized_message, type_support, message_wrapper);
            if (msg_type ==  "delphi_esr_msgs/msg/EsrTrack") {
                RadarMessageT radar_message = message_converter.getRadarMessage();
                if (calibrated_sensors.find(serialized_message->topic_name) == calibrated_sensors.end()) {
                    writeCalibratedSensor(radar_message.frame_id, std::vector<std::vector<float>>());
                    calibrated_sensors.insert({serialized_message->topic_name});
                }
            } else if (msg_type == "sensor_msgs/msg/CompressedImage") {
                CameraMessageT camera_message = message_converter.getCameraMessage();
            } else if (msg_type == "sensor_msgs/msg/PointCloud2") {
                LidarMessageT lidar_message = message_converter.getLidarMessage();
                if (calibrated_sensors.find(serialized_message->topic_name) == calibrated_sensors.end()) {
                    writeCalibratedSensor(lidar_message.frame_id, std::vector<std::vector<float>>());
                    calibrated_sensors.insert({serialized_message->topic_name});
                }
            } else if (msg_type == "nav_msgs/msg/Odometry") {
                OdometryMessageT odometry_message = message_converter.getOdometryMessage();
            } else if (msg_type == "sensor_msgs/msg/CameraInfo") {
                CameraCalibrationT camera_calibration = message_converter.getCameraCalibration();
                if (calibrated_sensors.find(serialized_message->topic_name) == calibrated_sensors.end()) {
                    writeCalibratedSensor(camera_calibration.frame_id, camera_calibration.intrinsic);
                    calibrated_sensors.insert({serialized_message->topic_name});
                }
            }
        }
    }
    
    
}

void Bag2Scenes::writeLog() {
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
    writeMap(log_token);
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

void Bag2Scenes::writeCalibratedSensor(std::string frame_id, std::vector<std::vector<float>> camera_intrinsic) {
    nlohmann::json calibrated_sensors;
    if (fs::exists("v1.0-mini/calibrated_sensor.json")) {
        std::ifstream calibrated_sensor_in("v1.0-mini/calibrated_sensor.json");
        calibrated_sensors = nlohmann::json::parse(calibrated_sensor_in);
        calibrated_sensor_in.close();
    }
    fs::path urdf_file = fs::path("..") / param_yaml_["BAG_INFO"]["URDF"].as<std::string>();
    pugi::xml_document urdf;
    if (!urdf.load_file(urdf_file.c_str())) {
        printf("Error loading URDF");
        exit(1);
    }
    pugi::xml_node joint = urdf.child("robot").find_child_by_attribute("type", "fixed");
    while (joint) {
        std::string channel = joint.child("child").attribute("link").value();
        if (channel == frame_id) {
            std::string translation = joint.child("origin").attribute("xyz").value();
            std::string rotation = joint.child("origin").attribute("rpy").value();            
            nlohmann::json calibrated_sensor;
            frame_info_[channel]["token"] = generateToken();
            calibrated_sensor["token"] = frame_info_[channel]["token"].as<std::string>();
            calibrated_sensor["sensor_token"] = writeSensor(channel);
            calibrated_sensor["translation"] = splitString(translation);
            std::vector<float> euler_angles = splitString(rotation);
            if (!euler_angles.size()) {
                euler_angles = {0.0, 0.0, 0.0};
            }
            Eigen::Quaternionf quat;
            quat = Eigen::AngleAxisf(euler_angles[0], Eigen::Vector3f::UnitX()) 
                    * Eigen::AngleAxisf(euler_angles[1], Eigen::Vector3f::UnitY())
                    * Eigen::AngleAxisf(euler_angles[2], Eigen::Vector3f::UnitZ());
            calibrated_sensor["rotation"] = quat.coeffs();
            calibrated_sensor["camera_intrinsic"] = camera_intrinsic;
            calibrated_sensors.push_back(calibrated_sensor);
            std::ofstream calibrated_sensor_out("v1.0-mini/calibrated_sensor.json");
            calibrated_sensor_out << std::setw(4) << calibrated_sensors << std::endl;
            calibrated_sensor_out.close();
        }
        joint = joint.next_sibling();
    }    
}

std::string Bag2Scenes::writeSensor(std::string channel) {
    nlohmann::json sensors;
    std::string sensor_token;
    if (!fs::exists("v1.0-mini/sensor.json")) {
        nlohmann::json sensor;
        sensor_token = generateToken();
        sensor["token"] = sensor_token;
        std::string modality = channel.substr(0, channel.find("_"));
        sensor["channel"] = channel;
        std::transform(modality.begin(), modality.end(), modality.begin(), ::tolower);
        sensor["modality"] = modality;
        std::string directory = frame_info_[channel]["name"].as<std::string>();
        std::transform(directory.begin(), directory.end(), directory.begin(), ::toupper);
        fs::create_directory(fs::path("v1.0-mini/samples") / directory);
        fs::create_directory(fs::path("v1.0-mini/sweeps") / directory);
        sensors.push_back(sensor);
        std::ofstream sensor_out("v1.0-mini/sensor.json");
        sensor_out << std::setw(4) << sensors << std::endl;
    } else {
        std::ifstream sensor_in("v1.0-mini/sensor.json");
        sensors = nlohmann::json::parse(sensor_in);
        sensor_in.close();
        for (nlohmann::json sensor : sensors) {
            if (sensor["channel"] == channel) return sensor["token"];
        }
        nlohmann::json sensor;
        sensor_token = generateToken();
        sensor["token"] = sensor_token;
        std::string modality = channel.substr(0, channel.find("_"));
        sensor["channel"] = channel;
        std::transform(modality.begin(), modality.end(), modality.begin(), ::tolower);
        sensor["modality"] = modality;
        std::string directory = frame_info_[channel]["name"].as<std::string>();
        std::transform(directory.begin(), directory.end(), directory.begin(), ::toupper);
        fs::create_directory(fs::path("v1.0-mini/samples") / directory);
        fs::create_directory(fs::path("v1.0-mini/sweeps") / directory);
        sensors.push_back(sensor);
        std::ofstream sensor_out("v1.0-mini/sensor.json");
        sensor_out << std::setw(4) << sensors << std::endl;
    }
    return sensor_token;
}

std::string Bag2Scenes::generateToken() {
    char token[17];
    for (int i = 0; i < 17; i++) {
        sprintf(token + i, "%x", rand() % 16);
    }
    return std::string(token);
}

std::vector<float> Bag2Scenes::splitString(std::string str) {
    std::string s;
    std::stringstream ss(str);
    std::vector<float> v;
    while (getline(ss, s, ' ')) {
        v.push_back(std::stof(s));
    }
    return v;
}