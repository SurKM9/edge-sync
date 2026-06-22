/**
 * @file sync_engine.hpp
 * @brief Temporal synchronisation engine that aligns IMU and camera streams.
 *
 * SyncEngine owns a dedicated worker thread that continuously drains an IMU queue
 * and a camera queue, then matches messages by timestamp so downstream fusion
 * receives temporally aligned pairs.
 */

#ifndef EDGE_SYNC_SYNC_SYNC_ENGINE_HPP
#define EDGE_SYNC_SYNC_SYNC_ENGINE_HPP

#include "edge_sync/core/sensor_types.hpp"
#include "edge_sync/concurrency/spsc_ring_buffer.hpp"
#include "edge_sync/fusion/fusion_strategy.hpp"
#include <atomic>
#include <thread>
#include <memory>

namespace edge_sync::sync
{

/**
 * @brief Consumes heterogeneous sensor queues and produces temporally aligned frames.
 *
 * The engine spins up a single background thread on start() that polls both the
 * IMU and camera SPSC queues. When a camera frame arrives, the engine searches
 * the buffered IMU samples for the closest timestamp and emits a synchronised
 * pair to the fusion layer.
 *
 * ### Ownership model
 * SyncEngine holds **non-owning references** to the two queues. The caller is
 * responsible for ensuring both queues outlive the engine.
 *
 * ### Thread model
 * ```
 * Ingestion threads (producers)
 *       │  ImuQueue (SPSC)
 *       ▼
 *  [ SyncEngine worker ]  ← single consumer for both queues
 *       │  CameraQueue (SPSC)
 *       ▲
 * Ingestion threads (producers)
 * ```
 *
 * @note Not copyable or movable — the worker thread holds a pointer to `this`.
 */
class SyncEngine
{
public:

    // Inject the strategy via a unique_ptr so the engine takes ownership of it
    explicit SyncEngine(std::unique_ptr<fusion::FusionStrategy> fusionStrategy);

    /**
     * @brief Stops the worker thread (if running) and destroys the engine.
     *
     * Calls stop() internally so callers do not need to do so explicitly before
     * destruction, but doing so is safe.
     */
    ~SyncEngine();

    SyncEngine(const SyncEngine&)            = delete; ///< Non-copyable.
    SyncEngine(SyncEngine&&)                 = delete; ///< Non-movable.
    SyncEngine& operator=(const SyncEngine&) = delete; ///< Non-copy-assignable.
    SyncEngine& operator=(SyncEngine&&)      = delete; ///< Non-move-assignable.

    /**
     * @brief Launches the background processing thread.
     *
     * Sets `m_isRunning` to `true` then spawns `m_workerThread` which
     * executes processingLoop(). Calling start() on an already-running engine
     * is undefined behaviour.
     */
    void start();

    /**
     * @brief Signals the worker thread to exit and blocks until it joins.
     *
     * Sets `m_isRunning` to `false` so processingLoop() exits on its next
     * iteration, then calls `m_workerThread.join()`. Safe to call from any
     * thread. No-op if the engine was never started.
     */
    void stop();

private:

    /**
     * @brief Main loop executed by the worker thread.
     *
     * Runs until `m_isRunning` is `false`. On each iteration:
     *  1. Drains available IMU messages from `m_imuQueue`.
     *  2. Checks `m_cameraQueue` for a new camera frame.
     *  3. If a frame is available, finds the IMU sample with the nearest
     *     timestamp and emits an aligned pair downstream.
     */
    void processingLoop();

    concurrency::SpscRingBuffer<core::ImuMessage*>&    m_imuQueue;    ///< Non-owning ref to the IMU ingestion queue.
    concurrency::SpscRingBuffer<core::CameraMessage*>& m_cameraQueue; ///< Non-owning ref to the camera ingestion queue.

    std::unique_ptr<fusion::FusionStrategy> m_fusionStrategy;

    std::atomic<bool> m_isRunning{false}; ///< Signals the worker thread to keep running; false triggers exit.
    std::thread       m_workerThread;     ///< Background thread executing processingLoop().
};

} // namespace edge_sync::sync

#endif // EDGE_SYNC_SYNC_SYNC_ENGINE_HPP
