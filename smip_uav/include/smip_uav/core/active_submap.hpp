#ifndef SMIP_ACTIVE_SUBMAP_HPP_
#define SMIP_ACTIVE_SUBMAP_HPP_

#include <Eigen/Geometry>

#include "core/types.hpp"
#include "core/surfel.hpp"

namespace smip_uav {

using pnw = PointNormalWeight;

struct ActiveSurfel {
    Surfel stable_surfel;
    Eigen::Vector3f S1{Eigen::Vector3f::Zero()}; // accumulated position
    Eigen::Matrix3f S2{Eigen::Matrix3f::Zero()}; // accumulated second moment matrix
    int32_t N{0};

    float stability{0.0f};

    bool stable{false}; // true if statistics will form a stable surfel

    void finalize() {
        // Compute the stable (if so) surfel based on statistics and alignment in local map
        
        if (stable) {
            // compute the surfel
            return;
        }

        return;
    }
};

struct ActiveSubmap {
    Eigen::Isometry3f T_world_submap;
    std::vector<ActiveSurfel> surfels;
    
    std::vector<pnw> support_cloud; // this could be a downsampled representative of the underlying geometry for fitting surfels


};


} // namespace smip_uav

#endif