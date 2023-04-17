#ifndef MESSAGE_TYPES_HPP
#define MESSAGE_TYPES_HPP

#define PCL_NO_PRECOMPILE
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>

struct SensorMessageT {
    unsigned long timestamp;
    std::string frame_id;
    virtual ~SensorMessageT() = default;
};

struct RadarPointT {
    float x;
    float y;
    float z;
    int dyn_prop;
    int id;
    float rcs;
    float vx;
    float vy;
    float vx_comp;
    float vy_comp;
    int is_quality_valid;
    int ambig_state;
    int x_rms;
    int y_rms;
    int invalid_state;
    int pdh0;
    int vx_rms;
    int vy_rms;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(RadarPointT,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (int, dyn_prop, dyn_prop)
                                    (int, id, id)
                                    (float, rcs, rcs)
                                    (float, vx, vx)
                                    (float, vy, vy)
                                    (float, vx_comp, vx_comp)
                                    (float, vy_comp, vy_comp)
                                    (int, is_quality_valid, is_quality_valid)
                                    (int, ambig_state, ambig_state)
                                    (int, x_rms, x_rms)
                                    (int, y_rms, y_rms)
                                    (int, invalid_state, invalid_state)
                                    (int, pdh0, pdh0)
                                    (int, vx_rms, vx_rms)
                                    (int, vy_rms, vy_rms)
)

struct RadarMessageT : SensorMessageT {
    pcl::PointCloud<RadarPointT> cloud;
};

struct LidarMessageT : SensorMessageT {
    pcl::PointCloud<pcl::PointXYZI> cloud;
};

struct CameraMessageT : SensorMessageT {
    cv::Mat image;
};

struct OdometryMessageT {
    unsigned long timestamp;
    std::vector<double> position;
    std::vector<double> orientation;
};

struct CameraCalibrationT {
    std::string frame_id;
    std::vector<std::vector<float>> intrinsic;
};


#endif
