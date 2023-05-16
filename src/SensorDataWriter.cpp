#include "rosbag2nuscenes/SensorDataWriter.hpp"

SensorDataWriter::SensorDataWriter(int num_workers) : file_queue_(MAX_QUEUE_SIZE) {
    if (num_workers <=0) num_workers = 1;
    for (int i = 0; i < num_workers; i++) {
        thread_vector_.push_back(std::thread {&SensorDataWriter::writeFile, this});
    }
}

void SensorDataWriter::writeRadarData(RadarMessageT msg, fs::path filename) {
    if (fs::exists(filename)) {
        return;
    }
    pcl::io::savePCDFileBinary(filename, msg.cloud);
}

void SensorDataWriter::writeLidarData(LidarMessageT msg, fs::path filename) {
    if (msg.cloud.empty()) {
        throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Input point cloud has no data!");
        exit(1);
    }
    int data_idx = 0;
    std::ostringstream oss;
    data_idx = static_cast<int> (oss.tellp ());
    int fd = pcl::io::raw_open (filename.c_str (), O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    if (fd < 0) {
        throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Error during open!");
        exit(1);
    }
    auto fields = pcl::getFields<pcl::PointXYZI> ();
    std::vector<int> fields_sizes;
    std::size_t fsize = 0;
    std::size_t data_size = 0;
    std::size_t nri = 0;
    // Compute the total size of the fields
    for (const auto &field : fields) {
        if (field.name == "_")
            continue;

        int fs = field.count * pcl::getFieldSize (field.datatype);
        fsize += fs;
        fields_sizes.push_back (fs);
        fields[nri++] = field;
    }
    fields.resize (nri);

    data_size = msg.cloud.size() * (fsize + 4);

    // Prepare the map
    // Allocate disk space for the entire file to prevent bus errors.
    if (pcl::io::raw_fallocate (fd, data_idx + data_size) != 0) {
        pcl::io::raw_close (fd);
        PCL_ERROR ("[pcl::PCDWriter::writeBinary] posix_fallocate errno: %d strerror: %s\n", errno, strerror (errno));

        throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Error during posix_fallocate ()!");
        exit(1);
    }

    char *map = static_cast<char*> (::mmap (nullptr, data_idx + data_size, PROT_WRITE, MAP_SHARED, fd, 0));
    if (map == reinterpret_cast<char*> (-1)) {
        pcl::io::raw_close (fd);
        throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Error during mmap ()!");
        exit(1);
    }
    // Copy the data
    char *out = &map[0] + data_idx;
    char zero[4] = {0, 0, 0, 0};
    for (const auto& point: msg.cloud) {
        int nrj = 0;
        for (const auto &field : fields) {
            memcpy (out, reinterpret_cast<const char*> (&point) + field.offset, fields_sizes[nrj]);
            out += fields_sizes[nrj++];
        }
        memcpy (out, reinterpret_cast<const char*> (&zero), 4);
        out += 4;
    }
    if (::munmap (map, (data_idx + data_size)) == -1) {
        pcl::io::raw_close (fd);
        throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Error during munmap ()!");
        exit(1);
    }
    // Close file
    pcl::io::raw_close (fd);
}

void SensorDataWriter::writeCameraData(CameraMessageT msg, fs::path filename) {
    cv::imwrite(filename, msg.image);
}

void SensorDataWriter::writeSensorData(SensorMessageT* msg, fs::path filename) {
    std::unique_lock<std::mutex> lck(queue_mutex_);
    if (file_queue_.size() == MAX_QUEUE_SIZE) {
        queue_full_.wait(lck);
    }
    file_queue_.push_back(std::pair {msg, filename});
    lck.unlock();
    queue_empty_.notify_one();
}


void SensorDataWriter::writeFile() {
    RadarMessageT* radar_msg;
    LidarMessageT* lidar_msg;
    CameraMessageT* camera_msg;
    while (!finished_) {
        std::unique_lock<std::mutex> lck(queue_mutex_);
        while(file_queue_.empty() && !finished_) {
            queue_empty_.wait(lck);
        }
        if (finished_) return;
        std::pair<SensorMessageT*, fs::path> data = file_queue_.front();
        file_queue_.pop_front();
        if ((radar_msg = dynamic_cast<RadarMessageT*> (std::get<0>(data)))) {
            writeRadarData(*radar_msg, std::get<1>(data));
            delete(radar_msg);
            radar_msg = nullptr;
        } else if ((lidar_msg = dynamic_cast<LidarMessageT*> (std::get<0>(data)))) {
            writeLidarData(*lidar_msg, std::get<1>(data));
            delete(lidar_msg);
            lidar_msg = nullptr;
        } else if ((camera_msg = dynamic_cast<CameraMessageT*> (std::get<0>(data)))){
            writeCameraData(*camera_msg, std::get<1>(data));
            delete(camera_msg);
            camera_msg = nullptr;
        }
        lck.unlock();
        queue_full_.notify_one();
    }
}


void SensorDataWriter::close() {
    finished_ = true;
    queue_empty_.notify_all();
    for (auto& t : thread_vector_) {
        t.join();
    }
}
