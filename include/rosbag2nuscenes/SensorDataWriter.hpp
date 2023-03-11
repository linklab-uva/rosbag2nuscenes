#ifndef SENSOR_DATA_WRITER_HPP
#define SENSOR_DATA_WRITER_HPP

#include "MessageTypes.hpp"

class SensorDataWriter {
    public:
        SensorDataWriter();

        void writeRadarData(RadarMessageT msg);

        void writeLidarData(LidarMessageT msg);

        void writeCameraData(CameraMessageT msg);

    private:
        void saveSensorData(void);

        // TODO: thread pool stuff
};



#endif