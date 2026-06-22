#include "edge_sync/fusion/complementary_filter.hpp"

namespace edge_sync::fusion
{

ComplementaryFilter::ComplementaryFilter(double alpha) : 
        m_alpha(alpha),
        m_currentEstimate(Eigen::Quaterniond::Identity()),
        m_lastTimestampNs(0)
{    
}

Eigen::Quaterniond ComplementaryFilter::process(const core::ImuMessage* imuMsg, 
                                                const core::CameraMessage* cameraMsg)
{
    double dtSeconds = 0.0;
    if(m_lastTimestampNs != 0)
    {
        dtSeconds = static_cast<double>(imuMsg->timestampNs - m_lastTimestampNs) * 1e-9;        
    }

    m_lastTimestampNs = imuMsg->timestampNs;

    Eigen::Vector3d gyroVector(imuMsg->gyroX, imuMsg->gyroY, imuMsg->gyroZ);

    Eigen::Quaterniond qGyroDelta;
    qGyroDelta.w() = 1.0;
    qGyroDelta.vec() = 0.5 * gyroVector * dtSeconds;
    qGyroDelta.normalize();

    Eigen::Quaterniond qPred = m_currentEstimate * qGyroDelta;
    qPred.normalize();

    Eigen::Quaterniond qCam(cameraMsg->orientation_w,
                            cameraMsg->orientation_x,
                            cameraMsg->orientation_y,
                            cameraMsg->orientation_z);

    m_currentEstimate = qCam.slerp(m_alpha, qPred);
    m_currentEstimate.normalize();
    
    return m_currentEstimate;
}
}   