#include "rosbag2nuscenes/Bag2Scenes.hpp"

Bag2Scenes::Bag2Scenes(const std::filesystem::path rosbag_dir, const std::filesystem::path param_file) {

}

void Bag2Scenes::writeScene() {

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