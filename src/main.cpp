/**
 * @file main.cpp
 * @brief End-to-end integration harness for the Edge Sync pipeline.
 *
 * Wires together the full pipeline in the correct dependency order:
 *   1. SPSC queues (transport layer)
 *   2. ComplementaryFilter (fusion strategy)
 *   3. SyncEngine (consumer / orchestrator)
 *
 * Then injects synthetic sensor readings to verify the temporal alignment
 * and fusion path before real hardware is connected.
 */

#include "edge_sync/fusion/complementary_filter.hpp"
#include "edge_sync/sync/sync_engine.hpp"
#include "edge_sync/core/sensor_types.hpp"
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

using namespace edge_sync;

int main()
{
    std::cout << "--- Edge Sync Pipeline Starting ---" << std::endl;

    // ---------------------------------------------------------------------------
    // TRANSPORT LAYER
    // IMU runs at ~200 Hz so 1024 slots gives ~5 seconds of headroom before the
    // ring wraps. Camera at ~20 Hz only needs 256 slots (~12 seconds headroom).
    // ---------------------------------------------------------------------------
    concurrency::SpscRingBuffer<core::ImuMessage*> imuQueue(1024);
    concurrency::SpscRingBuffer<core::CameraMessage*> cameraQueue(256);

    // ---------------------------------------------------------------------------
    // FUSION STRATEGY
    // Alpha = 0.98: trust the gyroscope for 98% of each update; pull 2% toward
    // the camera-derived orientation to correct long-term drift.
    // ---------------------------------------------------------------------------
    auto filter = std::make_unique<fusion::ComplementaryFilter>(0.98);

    // ---------------------------------------------------------------------------
    // SYNC ENGINE
    // Takes ownership of the filter via move. The engine starts its worker thread
    // only after start() is called, giving us time to push initial data first.
    // ---------------------------------------------------------------------------
    sync::SyncEngine engine(std::move(filter), imuQueue, cameraQueue);

    engine.start();
    std::cout << "[Main] Background SyncEngine thread is live." << std::endl;

    // ---------------------------------------------------------------------------
    // SYNTHETIC TELEMETRY
    // Three IMU samples spaced 1 ms apart (timestamps in nanoseconds):
    //   imu1 @ t=1,000,000 ns  (1.0 ms)
    //   imu2 @ t=2,000,000 ns  (2.0 ms)
    //   imu3 @ t=3,000,000 ns  (3.0 ms)
    //
    // accelZ = 9.81 m/s² simulates gravity on the Z axis (sensor at rest).
    // gyroX  = 0.1 rad/s simulates a slow roll rotation.
    // ---------------------------------------------------------------------------
    // The IMU insists we are completely stationary (gyro = 0, 0, 0)
    core::ImuMessage imu1{1000000, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0};
    core::ImuMessage imu2{2000000, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0};
    core::ImuMessage imu3{3000000, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0};
    
    // The Camera insists we just instantly turned 90-degrees left (w=0.707, z=0.707)
    core::CameraMessage cam1{2500000, "/mock/image_001.png", 0.7071, 0.0, 0.0, 0.7071};

    // 6. Fire the conflicting data into the queues
    std::cout << "[Main] Pushing conflicting telemetry..." << std::endl;
    imuQueue.push(&imu1);
    imuQueue.push(&imu2);
    cameraQueue.push(&cam1);
    imuQueue.push(&imu3);

    // Give the worker thread time to drain both queues and complete the fusion
    // cycle before we signal shutdown. 100 ms >> one 50 µs poll interval.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // ---------------------------------------------------------------------------
    // CLEAN SHUTDOWN
    // engine.stop() signals m_isRunning = false, then joins the worker thread.
    // The SyncEngine destructor also calls stop(), so this explicit call is
    // optional — included here for clarity in the harness.
    // ---------------------------------------------------------------------------
    std::cout << "[Main] Shutting down engine..." << std::endl;
    engine.stop();

    std::cout << "--- Pipeline Exited Cleanly ---" << std::endl;
    return 0;
}
