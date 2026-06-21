#ifndef EDGE_SYNC_FUSION_FUSION_STRATEGY_HPP
#define EDGE_SYNC_FUSION_FUSION_STRATEGY_HPP

#include "edge_sync/core/sensor_types.hpp"
#include <Eigen/Dense>

namespace edge_sync::fusion
{
class FusionStrategy
{
    public:

        virtual ~FusionStrategy() = default;

        virtual Eigen::Quaterniond process(const core::ImuMessage* imuMsg, const core::CameraMessage* cameraMsg) = 0; 
};
}

#endif // EDGE_SYNC_FUSION_FUSION_STRATEGY_HPP