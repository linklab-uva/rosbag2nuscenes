#include "rosbag2nuscenes/Bag2Scenes.hpp"
#include <iostream>

Bag2Scenes::Bag2Scenes(const fs::path rosbag_dir, const fs::path param_file) : 
odometry_bar_{
        indicators::option::BarWidth{80},
        indicators::option::ForegroundColor{indicators::Color::white},
        indicators::option::PrefixText{"Parsing Ego Odometry "},
        indicators::option::FontStyles{std::vector<indicators::FontStyle>{indicators::FontStyle::bold}}
},
sensor_data_bar_{
    indicators::option::BarWidth{80},
    indicators::option::ForegroundColor{indicators::Color::white},
    indicators::option::PrefixText{"Parsing Sensor Data  "},
    indicators::option::FontStyles{std::vector<indicators::FontStyle>{indicators::FontStyle::bold}}
},
progress_bars_(sensor_data_bar_, odometry_bar_) {
    // Read Parameter File
    try {
        param_yaml_ = YAML::LoadFile(param_file);
    } catch (std::exception e) {
        printf("Error reading %s\n", param_file.c_str());
        exit(1);
    }
    // Storage Options
    storage_options_.uri = rosbag_dir;
    storage_options_.storage_id = "sqlite3";
    // Converter Options
    converter_options_.input_serialization_format = "cdr";
    converter_options_.output_serialization_format = "cdr";
    // Open Bag and Store Metadata
    rosbag2_cpp::readers::SequentialReader reader;
    try {
        reader.open(storage_options_, converter_options_);
    } catch (std::exception e) {
        exit(1);
    }
    std::vector<rosbag2_storage::TopicMetadata> topic_types = reader.get_all_topics_and_types();
    bag_data_ = reader.get_metadata();
    for (std::vector<rosbag2_storage::TopicMetadata>::iterator itr  = topic_types.begin(); itr != topic_types.end(); itr++) {
        topic_to_type_.insert({itr->name, itr->type});
    }
    for (YAML::Node::iterator itr = param_yaml_["SENSOR_INFO"].begin(); itr != param_yaml_["SENSOR_INFO"].end(); itr++) {
        std::string sensor_name = itr->first.as<std::string>();
        frame_info_[itr->second["FRAME"].as<std::string>()]["previous_timestamp"] = 0;
        frame_info_[itr->second["FRAME"].as<std::string>()]["previous_token"] = "";
        frame_info_[itr->second["FRAME"].as<std::string>()]["next_token"] = generateToken();
        frame_info_[itr->second["FRAME"].as<std::string>()]["name"] = sensor_name;
        frame_info_[itr->second["FRAME"].as<std::string>()]["sensor_token"] = generateToken();
        std::string modality = sensor_name.substr(0, sensor_name.find("_"));
        std::string topic = itr->second["TOPIC"].as<std::string>();
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
    reader.close();
    bag_dir_ = rosbag_dir.parent_path().filename();
    srand(time(0));
    indicators::show_console_cursor(false);
    previous_sampled_timestamp_ = bag_data_.starting_time.time_since_epoch().count();
    previous_sample_token_ = "";
    next_sample_token_ = generateToken();
    nbr_samples_ = 0;
    waiting_timestamp_ = 0;
}

void Bag2Scenes::writeScene() {
    MessageConverter message_converter;
    scene_token_ = generateToken();
    if (!fs::exists("v1.0-mini/")) {
        fs::create_directory("v1.0-mini");
        fs::create_directory("samples");
        fs::create_directory("sweeps");
    }
    std::string log_token = writeLog();
    nlohmann::json scene;
    scene["token"] = scene_token_;
    scene["log_token"] = log_token;
    scene["first_sample_token"] = next_sample_token_;
    std::unordered_set<std::string> calibrated_sensors;
    nlohmann::json ego_poses;
    nlohmann::json sample_data;
    nlohmann::json scenes;
    if (fs::exists("v1.0-mini/ego_pose.json")) {
        std::ifstream ego_poses_in("v1.0-mini/ego_pose.json");
        ego_poses = nlohmann::json::parse(ego_poses_in);
        ego_poses_in.close();
    }
    if (fs::exists("v1.0-mini/sample.json")) {
        std::ifstream sample_in("v1.0-mini/sample.json");
        samples_ = nlohmann::json::parse(sample_in);
        sample_in.close();
    }
    if (fs::exists("v1.0-mini/sample_data.json")) {
        std::ifstream sample_data_in("v1.0-mini/sample_data.json");
        sample_data = nlohmann::json::parse(sample_data_in);
        sample_data_in.close();
    }
    if (fs::exists("v1.0-mini/scene.json")) {
        std::ifstream scene_in("v1.0-mini/scene.json");
        scenes = nlohmann::json::parse(scene_in);
        scene_in.close();
    }
    // Write odometry data
    std::thread ego_pose_thread(&Bag2Scenes::writeEgoPose, this, std::ref(ego_poses));
    // Write sensor data
    std::thread sensor_data_thread(&Bag2Scenes::writeSampleData, this, std::ref(sample_data));
    ego_pose_thread.join();
    sensor_data_thread.join();
    std::ofstream ego_poses_out("v1.0-mini/ego_pose.json");
    ego_poses_out << std::setw(4) << ego_poses << std::endl;
    ego_poses_out.close();
    std::ofstream sample_data_out("v1.0-mini/sample_data.json");
    sample_data_out << std::setw(4) << sample_data << std::endl;
    sample_data_out.close();
    samples_.back()["next"] = "";
    std::ofstream sample_out("v1.0-mini/sample.json");
    sample_out << std::setw(4) << samples_ << std::endl;
    sample_out.close();
    scene["nbr_samples"] = nbr_samples_;
    scene["last_sample_token"] = current_sample_token_;
    scene["name"] = bag_dir_;
    scene["description"] = param_yaml_["BAG_INFO"]["DESCRIPTION"].as<std::string>();
    scenes.push_back(scene);
    std::ofstream scene_out("v1.0-mini/scene.json");
    scene_out << std::setw(4) << scenes << std::endl;
    scene_out.close();
    writeTaxonomyFiles();
    indicators::show_console_cursor(true);
    std::cout << "Done." << std::endl;
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
    writeMap(log_token);
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

void Bag2Scenes::writeSampleData(nlohmann::json& previous_data) {
    int sensor_data_msgs = 0;
    for (rosbag2_storage::TopicInformation topic_data : bag_data_.topics_with_message_count) {
        if (std::find(topics_of_interest_.begin(), topics_of_interest_.end(), topic_data.topic_metadata.name) != topics_of_interest_.end()) {
            sensor_data_msgs += topic_data.message_count;
        }
    }
    sensor_data_bar_.set_option(indicators::option::MaxProgress{sensor_data_msgs});
    rosbag2_cpp::readers::SequentialReader reader;
    reader.open(storage_options_, converter_options_);
    rosbag2_storage::StorageFilter storage_filter{};
    storage_filter.topics = std::vector<std::string> {topics_of_interest_};
    reader.set_filter(storage_filter);
    rosbag2_cpp::SerializationFormatConverterFactory factory;
    std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer> cdr_deserializer = factory.load_deserializer("cdr");
    MessageConverter message_converter;
    SensorDataWriter data_writer;
    std::unordered_set<std::string> calibrated_sensors;
    fs::path filename;
    std::string filename_str;
    for (int i = 0; i < sensor_data_msgs; i++) {
        if (reader.has_next()) {
            auto serialized_message = reader.read_next();
            auto message_wrapper = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
            std::string msg_type = topic_to_type_[serialized_message->topic_name];
            message_converter.getROSMsg(msg_type, message_wrapper);
            auto library = rosbag2_cpp::get_typesupport_library(msg_type, "rosidl_typesupport_cpp");
            auto type_support = rosbag2_cpp::get_typesupport_handle(msg_type, "rosidl_typesupport_cpp", library);
            cdr_deserializer->deserialize(serialized_message, type_support, message_wrapper);
            sensor_data_bar_.set_option(indicators::option::PostfixText{
                std::to_string(i+1) + "/" + std::to_string(sensor_data_msgs)
            });
            progress_bars_.tick<0>();
            if (calibrated_sensors.size() == lidar_topics_.size() + radar_topics_.size() + camera_calibs_.size() && msg_type == "sensor_msgs/msg/CameraInfo") {
                continue;
            }
            nlohmann::json sample_data;
            if (msg_type ==  "delphi_esr_msgs/msg/EsrTrack") {
                RadarMessageT radar_message = message_converter.getRadarMessage();
                if (calibrated_sensors.find(serialized_message->topic_name) == calibrated_sensors.end()) {
                    writeCalibratedSensor(radar_message.frame_id, std::vector<std::vector<float>>());
                    calibrated_sensors.insert({serialized_message->topic_name});
                }
                filename = getFilename(radar_message.frame_id, radar_message.timestamp);
                data_writer.writeRadarData(radar_message, filename);
                sample_data["token"] = frame_info_[radar_message.frame_id]["next_token"].as<std::string>();
                sample_data["calibrated_sensor_token"] = frame_info_[radar_message.frame_id]["sensor_token"].as<std::string>();
                sample_data["ego_pose_token"] = getClosestEgoPose(radar_message.timestamp);
                sample_data["height"] = 0;
                sample_data["width"] = 0;
                sample_data["timestamp"] = ((double)radar_message.timestamp) * pow(10, -8);
                sample_data["prev"] = frame_info_[radar_message.frame_id]["previous_token"].as<std::string>();
                frame_info_[radar_message.frame_id]["previous_token"] = frame_info_[radar_message.frame_id]["next_token"].as<std::string>();
                frame_info_[radar_message.frame_id]["next_token"] = generateToken();
                sample_data["next"] = frame_info_[radar_message.frame_id]["next_token"].as<std::string>();
            } else if (msg_type == "sensor_msgs/msg/CompressedImage") {
                CameraMessageT camera_message = message_converter.getCameraMessage();
                filename = getFilename(camera_message.frame_id, camera_message.timestamp);
                data_writer.writeCameraData(camera_message, filename);
                sample_data["token"] = frame_info_[camera_message.frame_id]["next_token"].as<std::string>();
                sample_data["calibrated_sensor_token"] = frame_info_[camera_message.frame_id]["sensor_token"].as<std::string>();
                sample_data["ego_pose_token"] = getClosestEgoPose(camera_message.timestamp);
                sample_data["height"] = camera_message.image.rows;
                sample_data["width"] = camera_message.image.cols;
                sample_data["timestamp"] = ((double)camera_message.timestamp) * pow(10, -8);
                sample_data["prev"] = frame_info_[camera_message.frame_id]["previous_token"].as<std::string>();
                frame_info_[camera_message.frame_id]["previous_token"] = frame_info_[camera_message.frame_id]["next_token"].as<std::string>();
                frame_info_[camera_message.frame_id]["next_token"] = generateToken();
                sample_data["next"] = frame_info_[camera_message.frame_id]["next_token"].as<std::string>();
            } else if (msg_type == "sensor_msgs/msg/PointCloud2") {
                LidarMessageT lidar_message = message_converter.getLidarMessage();
                if (calibrated_sensors.find(serialized_message->topic_name) == calibrated_sensors.end()) {
                    writeCalibratedSensor(lidar_message.frame_id, std::vector<std::vector<float>>());
                    calibrated_sensors.insert({serialized_message->topic_name});
                }
                filename = getFilename(lidar_message.frame_id, lidar_message.timestamp);
                data_writer.writeLidarData(lidar_message, filename);
                sample_data["token"] = frame_info_[lidar_message.frame_id]["next_token"].as<std::string>();
                sample_data["calibrated_sensor_token"] = frame_info_[lidar_message.frame_id]["sensor_token"].as<std::string>();
                sample_data["ego_pose_token"] = getClosestEgoPose(lidar_message.timestamp);
                sample_data["height"] = 0;
                sample_data["width"] = 0;
                sample_data["timestamp"] = ((double)lidar_message.timestamp) * pow(10, -8);
                sample_data["prev"] = frame_info_[lidar_message.frame_id]["previous_token"].as<std::string>();
                frame_info_[lidar_message.frame_id]["previous_token"] = frame_info_[lidar_message.frame_id]["next_token"].as<std::string>();
                frame_info_[lidar_message.frame_id]["next_token"] = generateToken();
                sample_data["next"] = frame_info_[lidar_message.frame_id]["next_token"].as<std::string>();
            } else if (msg_type == "sensor_msgs/msg/CameraInfo") {
                CameraCalibrationT camera_calibration = message_converter.getCameraCalibration();
                if (calibrated_sensors.find(serialized_message->topic_name) == calibrated_sensors.end()) {
                    writeCalibratedSensor(camera_calibration.frame_id, camera_calibration.intrinsic);
                    calibrated_sensors.insert({serialized_message->topic_name});
                }
                continue;
            }
            sample_data["sample_token"] = current_sample_token_;
            filename_str = filename.u8string();
            sample_data["filename"] = filename_str;
            sample_data["fileformat"] = filename_str.substr(filename_str.find('.')+1);
            sample_data["is_key_frame"] = (bool) (filename_str.find("samples") != std::string::npos);
            previous_data.push_back(sample_data);
        }
    }
}

void Bag2Scenes::writeEgoPose(nlohmann::json& previous_poses) {
    std::string odometry_topic = param_yaml_["BAG_INFO"]["ODOM_TOPIC"].as<std::string>();
    int odometry_msgs = 0;
    for (rosbag2_storage::TopicInformation topic_data : bag_data_.topics_with_message_count) {
        if (topic_data.topic_metadata.name == odometry_topic) {
            odometry_msgs = topic_data.message_count;
        }
    }
    odometry_bar_.set_option(indicators::option::MaxProgress{odometry_msgs});
    rosbag2_cpp::readers::SequentialReader reader;
    reader.open(storage_options_, converter_options_);
    rosbag2_storage::StorageFilter storage_filter{};
    storage_filter.topics = std::vector<std::string> {odometry_topic};
    reader.set_filter(storage_filter);
    rosbag2_cpp::SerializationFormatConverterFactory factory;
    std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer> cdr_deserializer = factory.load_deserializer("cdr");
    MessageConverter message_converter;
    for (int i = 0; i < odometry_msgs; i++) {
        if (reader.has_next()) {
            auto serialized_message = reader.read_next();
            auto message_wrapper = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
            std::string msg_type = topic_to_type_[serialized_message->topic_name];
            message_converter.getROSMsg(msg_type, message_wrapper);
            auto library = rosbag2_cpp::get_typesupport_library(msg_type, "rosidl_typesupport_cpp");
            auto type_support = rosbag2_cpp::get_typesupport_handle(msg_type, "rosidl_typesupport_cpp", library);
            cdr_deserializer->deserialize(serialized_message, type_support, message_wrapper);
            OdometryMessageT odometry_message = message_converter.getOdometryMessage();
            nlohmann::json new_pose;
            new_pose["token"] = generateToken();
            new_pose["timestamp"] = ((double)odometry_message.timestamp) * pow(10, -8);
            new_pose["rotation"] = odometry_message.orientation;
            new_pose["translation"] = odometry_message.position;
            previous_poses.push_back(new_pose);
            std::unique_lock<std::mutex> lck(ego_pose_mutex_);
            ego_pose_queue_.push_back(std::pair {odometry_message.timestamp, new_pose["token"]});
            if (waiting_timestamp_ && odometry_message.timestamp > waiting_timestamp_) {
                waiting_timestamp_ = 0;
                ego_pose_ready_.notify_all();
            }
        }
        odometry_bar_.set_option(indicators::option::PostfixText{
            std::to_string(i+1) + "/" + std::to_string(odometry_msgs)
        });
        progress_bars_.tick<1>();
    }
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
            calibrated_sensor["token"] = frame_info_[channel]["sensor_token"].as<std::string>();
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
            std::vector<float> wxyz = {quat.w(), quat.x(), quat.y(), quat.z()};
            calibrated_sensor["rotation"] = wxyz;
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
        fs::create_directory(fs::path("samples") / directory);
        fs::create_directory(fs::path("sweeps") / directory);
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
        fs::create_directory(fs::path("samples") / directory);
        fs::create_directory(fs::path("sweeps") / directory);
        sensors.push_back(sensor);
        std::ofstream sensor_out("v1.0-mini/sensor.json");
        sensor_out << std::setw(4) << sensors << std::endl;
    }
    return sensor_token;
}

void Bag2Scenes::writeTaxonomyFiles() {
    nlohmann::json category;
    nlohmann::json attribute;
    nlohmann::json visibility;
    nlohmann::json instance;
    nlohmann::json annotation;
    annotation["token"] = generateToken();
    if (!fs::exists("v1.0-mini/category.json")) {
        category["token"] = generateToken();
        category["name"] = "";
        category["description"] = "";
        nlohmann::json categories;
        categories.push_back(category);
        std::ofstream category_out("v1.0-mini/category.json");
        category_out << std::setw(4) << categories << std::endl;
        category_out.close();
    }
    if (!fs::exists("v1.0-mini/attribute.json")) {
        attribute["token"] = generateToken();
        attribute["name"] = "";
        attribute["description"] = "";
        nlohmann::json attributes;
        attributes.push_back(attribute);
        std::ofstream attribute_out("v1.0-mini/attribute.json");
        attribute_out << std::setw(4) << attributes << std::endl;
        attribute_out.close();
    }
    if (!fs::exists("v1.0-mini/visibility.json")) {
        visibility["token"] = "1";
        visibility["description"] = "";
        visibility["level"] = "";
        nlohmann::json visibilities;
        visibilities.push_back(visibility);
        std::ofstream visibility_out("v1.0-mini/visibility.json");
        visibility_out << std::setw(4) << visibilities << std::endl;
        visibility_out.close();
    }
    if (!fs::exists("v1.0-mini/instance.json")) {
        instance["token"] = generateToken();
        instance["category_token"] = category["token"];
        instance["nbr_annotations"] = 0;
        instance["first_annotation_token"] = annotation["token"];
        instance["last_annotation_token"] = annotation["token"];
        nlohmann::json instances;
        instances.push_back(instance);
        std::ofstream instance_out("v1.0-mini/instance.json");
        instance_out << std::setw(4) << instances << std::endl;
        instance_out.close();
    }
    if (!fs::exists("v1.0-mini/sample_annotation.json")) {
        annotation["sample_token"] = current_sample_token_;
        annotation["instance_token"] = instance["token"];
        annotation["visibility_token"] = visibility["token"];
        annotation["attribute_tokens"] = std::vector<std::string> {attribute["token"]};
        annotation["translation"] = std::vector<float> {0.0, 0.0, 0.0};
        annotation["size"] = std::vector<float> {0.0, 0.0, 0.0};
        annotation["rotation"] = std::vector<float> {1.0, 0.0, 0.0, 0.0};
        annotation["prev"] = "";
        annotation["next"] = "";
        annotation["num_lidar_pts"] = 0;
        annotation["num_radar_pts"] = 0;
        nlohmann::json annotations;
        annotations.push_back(annotation);
        std::ofstream annotation_out("v1.0-mini/sample_annotation.json");
        annotation_out << std::setw(4) << annotations << std::endl;
        annotation_out.close();
    }
}

std::string Bag2Scenes::generateToken() {
    char token[33];
    for (int i = 0; i < 33; i++) {
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

fs::path Bag2Scenes::getFilename(std::string channel, unsigned long timestamp) {
    std::string directory = frame_info_[channel]["name"].as<std::string>();
    std::string modality = directory.substr(0, directory.find("_"));
    std::transform(modality.begin(), modality.end(), modality.begin(), ::tolower);
    std::unique_ptr<char[]> buf;
    size_t size;
    std::string extension;
    std::string base_dir;
    if (modality == "lidar") {
        extension = ".pcd.bin";
    } else if (modality == "camera") {
        extension = ".jpg";
    } else if (modality == "radar") {
        extension = ".pcd";
    }
    if (is_key_frame(channel, timestamp)) {
        base_dir = "samples";
    } else {
        base_dir = "sweeps";
    }
    int size_s = std::snprintf( nullptr, 0, "%s/%s/%s__%s__%lu%s", base_dir.c_str(), directory.c_str(),bag_dir_.c_str(), directory.c_str(), timestamp, extension.c_str()) + 1; // Terminate with '\0'
    size = static_cast<size_t>( size_s );
    buf = std::unique_ptr<char[]>( new char[ size ] );
    std::snprintf( buf.get(), size, "%s/%s/%s__%s__%lu%s", base_dir.c_str(), directory.c_str(),bag_dir_.c_str(), directory.c_str(), timestamp, extension.c_str());
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

bool Bag2Scenes::is_key_frame(std::string channel, unsigned long timestamp) {
    if (previous_sampled_timestamp_ == (unsigned long) bag_data_.starting_time.time_since_epoch().count() || (long) timestamp - (long) previous_sampled_timestamp_ > 5 * pow(10, 8)) {
        previous_sampled_timestamp_ += 5 * pow(10,8);
        nbr_samples_++;
        nlohmann::json sample;
        current_sample_token_ = next_sample_token_;
        sample["token"] = current_sample_token_;
        sample["timestamp"] = ((double) previous_sampled_timestamp_) * pow(10,-8);
        sample["scene_token"] = scene_token_;
        sample["prev"] = previous_sample_token_;
        previous_sample_token_ = next_sample_token_;
        next_sample_token_ = generateToken();
        sample["next"] = next_sample_token_;
        samples_.push_back(sample);
        sensors_sampled_.clear();
        sensors_sampled_.insert({channel});
        return true;
    } else if (sensors_sampled_.find(channel) == sensors_sampled_.end()) {
        sensors_sampled_.insert({channel});
        return true;
    }
    return false;
}

std::string Bag2Scenes::getClosestEgoPose(unsigned long timestamp) {
    std::unique_lock<std::mutex> lck(ego_pose_mutex_);
    while (ego_pose_queue_.size() == 0) {
        waiting_timestamp_ = timestamp;
        ego_pose_ready_.wait(lck);
    }
    while (std::get<0>(ego_pose_queue_.back()) < timestamp) {
        waiting_timestamp_ = timestamp;
        ego_pose_ready_.wait(lck);
    }
    std::string previous_token;
    unsigned long previous_time_difference;
    for (unsigned long i = 0; i < ego_pose_queue_.size(); i++) {
        if (std::get<0>(ego_pose_queue_[i]) >= timestamp) {
            std::string return_token;
            if (std::get<0>(ego_pose_queue_[i]) - timestamp < previous_time_difference) {
                return_token = std::get<1>(ego_pose_queue_[i]);
            } else {
                return_token = previous_token;
            }
            ego_pose_queue_.erase(ego_pose_queue_.cbegin(), ego_pose_queue_.cbegin() + i);
            ego_pose_mutex_.unlock();
            return return_token;
        }
        previous_time_difference = timestamp - std::get<0>(ego_pose_queue_[i]);
        previous_token = std::get<1>(ego_pose_queue_[i]);
    }
    printf("Thread synchronization failed");
    exit(1);
}