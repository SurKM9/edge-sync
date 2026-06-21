/**
 * @file fusion_strategy.hpp
 * @brief Abstract interface for sensor fusion algorithms.
 *
 * Defines the contract that every fusion strategy must satisfy. Concrete
 * implementations (e.g., complementary filter, EKF, VIO) are injected into
 * the pipeline via this interface, keeping SyncEngine decoupled from any
 * specific algorithm.
 */

#ifndef EDGE_SYNC_FUSION_FUSION_STRATEGY_HPP
#define EDGE_SYNC_FUSION_FUSION_STRATEGY_HPP

#include "edge_sync/core/sensor_types.hpp"
#include <Eigen/Dense>

namespace edge_sync::fusion
{

/**
 * @brief Strategy interface for fusing an aligned (IMU, camera) pair into an orientation estimate.
 *
 * Follows the **Strategy** design pattern: the fusion algorithm is expressed as a
 * family of interchangeable objects that all satisfy this interface. SyncEngine
 * holds a pointer to FusionStrategy and calls process() at every aligned frame,
 * with no knowledge of which algorithm is underneath.
 *
 * ### Implementing a concrete strategy
 * ```cpp
 * class ComplementaryFilter : public FusionStrategy {
 * public:
 *     Eigen::Quaterniond process(const core::ImuMessage*    imu,
 *                                const core::CameraMessage* cam) override {
 *         // blend gyro integration with visual orientation correction
 *     }
 * };
 * ```
 *
 * @note Implementations must be real-time safe if called from SyncEngine's
 *       hot path: no heap allocation, no blocking I/O, bounded execution time.
 */
class FusionStrategy
{
public:

    /// Virtual destructor so concrete strategies are destroyed correctly through a base pointer.
    virtual ~FusionStrategy() = default;

    /**
     * @brief Fuses a temporally aligned IMU and camera sample into an orientation quaternion.
     *
     * Called by SyncEngine once per aligned pair, on the worker thread. The
     * returned quaternion represents the estimated orientation of the sensor
     * platform in world space at the time of the camera frame.
     *
     * @param imuMsg    IMU sample closest in time to the camera frame. Never null
     *                  when called from SyncEngine (the engine guards this).
     * @param cameraMsg Camera frame that triggered this fusion cycle. Never null.
     * @return Unit quaternion representing the fused orientation estimate.
     */
    virtual Eigen::Quaterniond process(const core::ImuMessage*    imuMsg,
                                       const core::CameraMessage* cameraMsg) = 0;
};

} // namespace edge_sync::fusion

#endif // EDGE_SYNC_FUSION_FUSION_STRATEGY_HPP
