/**
 * @file complementary_filter.hpp
 * @brief Complementary filter implementation of the FusionStrategy interface.
 *
 * Blends high-frequency gyroscope integration from the IMU with a low-frequency
 * orientation correction derived from the camera to produce a continuous,
 * drift-corrected orientation estimate.
 */

#ifndef EDGE_SYNC_FUSION_COMPLEMENTARY_FILTER_HPP_
#define EDGE_SYNC_FUSION_COMPLEMENTARY_FILTER_HPP_

#include "edge_sync/fusion/fusion_strategy.hpp"
#include <Eigen/Dense>
#include <cstdint>

namespace edge_sync::fusion
{

/**
 * @brief Orientation estimator that fuses IMU gyroscope data with camera corrections.
 *
 * ### Algorithm
 * On each call to process(), the filter performs two steps:
 *
 * **1. Gyroscope integration (predict)**
 * The angular velocity from the IMU is integrated over the elapsed time `dt`
 * to produce a short-term orientation delta:
 * ```
 * q_gyro = q_prev ⊗ exp(0.5 * ω * dt)
 * ```
 * This is accurate over short intervals but drifts unboundedly over time.
 *
 * **2. Camera correction (update)**
 * An absolute orientation derived from the camera frame is blended in using
 * the complementary weights `alpha` (trust gyro) and `1 - alpha` (trust camera):
 * ```
 * q_fused = slerp(q_camera, q_gyro, alpha)
 * ```
 * A higher `alpha` trusts the gyroscope more; a lower value pulls the estimate
 * toward the camera-derived orientation more aggressively.
 *
 * ### Tuning `alpha`
 * | Value | Effect |
 * |-------|--------|
 * | 0.98 (default) | Smooth, low noise; slow to correct large drift |
 * | 0.80           | Faster drift correction; more susceptible to camera noise |
 *
 * @note Not thread-safe. Must be called exclusively from SyncEngine's worker thread.
 */
class ComplementaryFilter : public FusionStrategy
{
public:

    /**
     * @brief Constructs the filter with an initial identity orientation.
     *
     * @param alpha Blending coefficient in the range (0, 1). Weight given to the
     *              gyroscope integration relative to the camera correction.
     *              Defaults to 0.98.
     */
    explicit ComplementaryFilter(double alpha = 0.98);

    /// Default virtual destructor; satisfies the FusionStrategy interface contract.
    ~ComplementaryFilter() override = default;

    ComplementaryFilter(const ComplementaryFilter&)            = delete; ///< Non-copyable — holds mutable filter state.
    ComplementaryFilter(ComplementaryFilter&&)                 = delete; ///< Non-movable.
    ComplementaryFilter& operator=(const ComplementaryFilter&) = delete; ///< Non-copy-assignable.
    ComplementaryFilter& operator=(ComplementaryFilter&&)      = delete; ///< Non-move-assignable.

    /**
     * @brief Fuses one aligned (IMU, camera) pair and returns the updated orientation.
     *
     * Integrates angular velocity from @p imuMsg over `dt` since the last call,
     * then blends the result with the camera-derived orientation using `m_alpha`.
     * Updates `m_currentEstimate` and `m_lastTimestampNs` in place.
     *
     * @param imuMsg    IMU sample for this fusion cycle. Must not be null.
     * @param cameraMsg Camera frame for this fusion cycle. Must not be null.
     * @return The updated orientation as a unit quaternion in world space.
     */
    Eigen::Quaterniond process(const core::ImuMessage*    imuMsg,
                               const core::CameraMessage* cameraMsg) override;

private:

    double m_alpha; ///< Gyroscope trust weight in [0, 1]. Higher = smoother but slower correction.

    Eigen::Quaterniond m_currentEstimate; ///< Running orientation estimate, updated each process() call.

    /// Timestamp of the last processed IMU sample (nanoseconds). Used to compute dt.
    /// Initialised to 0; the first call to process() skips integration and seeds the estimate.
    uint64_t m_lastTimestampNs;
};

} // namespace edge_sync::fusion

#endif // EDGE_SYNC_FUSION_COMPLEMENTARY_FILTER_HPP
