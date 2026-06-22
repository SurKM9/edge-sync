/**
 * @file sync_engine.cpp
 * @brief Implementation of SyncEngine — lifecycle management and processing loop.
 */

#include "edge_sync/sync/sync_engine.hpp"
#include <iostream>

namespace edge_sync::sync
{

SyncEngine::SyncEngine(std::unique_ptr<fusion::FusionStrategy> fusionStrategy,
    concurrency::SpscRingBuffer<core::ImuMessage*>& imuQueue,
    concurrency::SpscRingBuffer<core::CameraMessage*>& cameraQueue)
    : m_fusionStrategy(std::move(fusionStrategy)),
      m_imuQueue(imuQueue),
      m_cameraQueue(cameraQueue),
      m_isRunning(false)
{
    // Bind queues but defer thread creation until start() is called explicitly.
    // This lets the caller finish setting up producers before the consumer is live.
}

SyncEngine::~SyncEngine()
{
    // RAII guarantee: even if the caller forgets to call stop(), the thread is
    // always joined before the object's memory is released, preventing a detached
    // thread from accessing destroyed members.
    stop();
}

void SyncEngine::start()
{
    bool expected = false;

    // compare_exchange_strong atomically checks if m_isRunning == false, and only
    // if so, flips it to true and spawns the thread. This makes start() idempotent:
    // a second concurrent call will fail the exchange and do nothing.
    if (m_isRunning.compare_exchange_strong(expected, true))
    {
        m_workerThread = std::thread(&SyncEngine::processingLoop, this);
    }
}

void SyncEngine::stop()
{
    bool expected = true;

    // Atomically flip m_isRunning from true → false. If another thread already
    // called stop(), the exchange fails and we skip the join — preventing a
    // double-join on the same thread, which is undefined behaviour.
    if (m_isRunning.compare_exchange_strong(expected, false))
    {
        if (m_workerThread.joinable())
        {
            // Block until processingLoop() observes m_isRunning == false and exits.
            m_workerThread.join();
        }
    }
}

void SyncEngine::processingLoop()
{
    core::ImuMessage* latestImu{nullptr};

    // relaxed is sufficient here: we only need to observe our own store from start(),
    // and we don't need to synchronise any other memory with this flag check.
    while (m_isRunning.load(std::memory_order_relaxed))
    {
        // [HOT PATH] Temporal alignment logic goes here:
        //   1. Drain m_imuQueue into a local ring of recent IMU samples.
        //   2. Pop one CameraMessage* from m_cameraQueue.
        //   3. Binary-search the IMU ring for the sample closest in timestampNs.
        //   4. Emit the aligned (ImuMessage*, CameraMessage*) pair downstream.
        core::ImuMessage* imuMsg{nullptr};
        
        while(m_imuQueue.pop(imuMsg))
        {
            latestImu = imuMsg;
        }

        core::CameraMessage* cameraMsg{nullptr};

        if(m_cameraQueue.pop(cameraMsg))
        {
            if(latestImu != nullptr)
            {
                // [FUSION PIPELINE ENTRY POINT]
                // The temporal lock is achieved. Route the paired data into the math node.
                if (m_fusionStrategy) 
                {
                    Eigen::Quaterniond currentOrientation = m_fusionStrategy->process(latestImu, cameraMsg);
                    
                   // Add this to physically see the math engine working!
                   std::cout << "[Fusion Node] Temporal lock achieved! Fused Orientation (w, x, y, z): "
                        << currentOrientation.w() << ", "
                        << currentOrientation.x() << ", "
                        << currentOrientation.y() << ", "
                        << currentOrientation.z() << std::endl;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
}

} // namespace edge_sync::sync
