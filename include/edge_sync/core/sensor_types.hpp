/**
 * @file sensor_types.hpp
 * @brief Plain-Old-Data (POD) message structs for sensor telemetry.
 *
 * All types are intentionally kept as simple aggregates so they can be:
 *  - Stored directly inside ObjectPool<T> nodes without virtual-dispatch overhead.
 *  - Passed through SpscRingBuffer<T*> with a single pointer copy.
 *  - Trivially serialised / deserialised for logging or replay.
 */

#ifndef EDGE_SYNC_CORE_SENSOR_TYPES_HPP
#define EDGE_SYNC_CORE_SENSOR_TYPES_HPP

#include <cstdint>
#include <string>

namespace edge_sync::core
{

/**
 * @brief POD message carrying a single IMU sample.
 *
 * Designed for high-frequency publication (≥200 Hz). The layout is chosen so
 * the entire struct fits in one 64-byte L1 cache line:
 *
 * | Field        | Bytes |
 * |--------------|-------|
 * | timestampNs  |  8    |
 * | accelX/Y/Z   | 24    |
 * | gyroX/Y/Z    | 24    |
 * | **Total**    | **56**|
 */
struct ImuMessage
{
    int64_t timestampNs; ///< Monotonic capture timestamp in nanoseconds.
    double  accelX;      ///< Linear acceleration along the X axis (m/s²).
    double  accelY;      ///< Linear acceleration along the Y axis (m/s²).
    double  accelZ;      ///< Linear acceleration along the Z axis (m/s²).
    double  gyroX;       ///< Angular velocity around the X axis (rad/s).
    double  gyroY;       ///< Angular velocity around the Y axis (rad/s).
    double  gyroZ;       ///< Angular velocity around the Z axis (rad/s).
};

/**
 * @brief Lightweight message referencing a single camera frame by file path.
 *
 * For the current simulation stage the actual pixel data is not transferred
 * inline; only the path to the image on disk is stored. This avoids copying
 * large buffers through the pipeline until a direct-memory / DMA path is wired.
 *
 * @note This struct is not POD because @c std::string is not trivially copyable.
 *       Do not place it inside an ObjectPool that assumes trivial construction.
 */
struct CameraMessage
{
    int64_t     timestampNs; ///< Monotonic capture timestamp in nanoseconds.
    std::string imagePath;   ///< Absolute or relative path to the captured frame.
};

} // namespace edge_sync::core

#endif // EDGE_SYNC_CORE_SENSOR_TYPES_HPP
