#ifndef SMIP_TYPES_HPP_
#define SMIP_TYPES_HPP_

#include <cstdint>
#include <Eigen/Geometry>

namespace smip_uav {

// Submap identifier
using SubmapId = uint32_t;
static constexpr SubmapId kInvalidSubmapId = UINT32_MAX;

// StampedPose: Sensor pose in world frame with associated uncertainty
struct StampedPose {
    Eigen::Isometry3f T_world; // sensor-in-world
    Eigen::Matrix<float,6,6> cov {Eigen::Matrix<float,6,6>::Zero()}; // 0: trusted
    int64_t stamp_ns{0};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace smip_uav

#endif