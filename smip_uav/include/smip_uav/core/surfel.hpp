#ifndef SMIP_SURFEL_HPP_
#define SMIP_SURFEL_HPP_

#include <Eigen/Core>

namespace smip_uav {

struct Surfel {
    Eigen::Vector3f position{Eigen::Vector3f::Zero()};
    Eigen::Vector3f normal{Eigen::Vector3f::Zero()};

    Eigen::Matrix3f covariance{Eigen::Matrix3f::Zero()}; // Measurement covariance
    Eigen::Matrix3f shape{Eigen::Matrix3f::Zero()}; // spatial-extend matrix
    
    float confidence{0.0f};
    float weight{0.0f};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace smip_uav

#endif