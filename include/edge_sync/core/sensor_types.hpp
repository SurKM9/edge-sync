#ifndef EDGE_SYNC_CORE_SENSOR_TYPES_HPP
#define EDGE_SYNC_CORE_SENSOR_TYPES_HPP

#include <cstdint>
#include <string>

namespace edge_sync::core
{

/**
 * @brief Plain Old Data (POD) struct for high-frequency IMU telemetry.
 * Memory footprint: 8 bytes (timestamp) + 24 bytes (accel) + 24 bytes (gyro) = 56 bytes.
 * Easily fits within a single 64-byte L1 cache line.
 */
struct ImuMessage {
    int64_t timestampNs;
    double accelX;
    double accelY;
    double accelZ; 
    double gyroX;
    double gyroY;
    double gyroZ;  
};

/**
 * @brief MVP struct for Camera telemetry. 
 * For this simulation, we pass the file path to the image to represent a frame.
 */
struct CameraMessage 
{
    int64_t timestampNs;
    std::string imagePath; 
};

}

#endif // EDGE_SYNC_CORE_SENSOR_TYPES_HPP