#ifndef SMIP_ACTIVE_MAP_HPP_
#define SMIP_ACTIVE_MAP_HPP_

#include <unordered_map>
#include <vector>
#include <cstdint>
#include <Eigen/Geometry>

#include "core/types.hpp"
#include "core/frame.hpp"
#include "core/surfel.hpp"
#include "core/frozen_submap.hpp"

namespace smip_uav {

class ActiveMap {
public:
    struct Config {
        float voxel_size{0.5f};

        float min_normal_dot{0.8f};
        float max_point_to_plane_m{0.05f};
        float max_mahalanobis_sq{9.0f};

        uint32_t maturity_obs_count{5};
        uint32_t max_unobserved_frames{100};

        float omega_regularization{1e-3f};
    };

    ActiveMap(const Eigen::Isometry3f& T_origin_world, int64_t stamp_ns);
    ActiveMap(const Eigen::Isometry3f& T_origin_world, int64_t stamp_ns, const Config& cfg);
    ActiveMap(const ActiveMap&) = delete;
    ActiveMap& operator=(const ActiveMap&) = delete;

    void register_frame(const Frame& frame, const StampedPose& stamped_pose);
    FrozenSubmap freeze(int64_t stamp_ns_end);

    uint32_t frame_count() const { return frame_count_; }
    float accumulated_translation() const { return accumulated_translation_; }
    float accumulated_rotation() const { return accumulated_rotation_rad_; }
    size_t surfel_count() const { return total_surfel_count_; }

    const Eigen::Isometry3f& T_origin_world() const { return T_origin_world_; }
    const Eigen::Isometry3f& T_latest_world() const { return T_latest_world_; }

private:
    struct VoxelKey {
        int32_t x, y, z;
        bool operator==(const VoxelKey& o) const {
            return x == o.x && y == o.y && z == o.z;
        }
    };

    struct VoxelKeyHash {
        size_t operator()(const VoxelKey& k) const noexcept {
            size_t h = 2166136261u;
            auto mix = [&](uint32_t v) {
                h ^= static_cast<size_t>(v);
                h *= 16777619u;
            };
            mix(static_cast<uint32_t>(k.x));
            mix(static_cast<uint32_t>(k.y));
            mix(static_cast<uint32_t>(k.z));
            return h;
        }
    };

    struct FusionState {
        Eigen::Matrix3f Omega{Eigen::Matrix3f::Zero()}; // sum R_i^{-1}
        Eigen::Vector3f xi{Eigen::Vector3f::Zero()};    // sum R_i^{-1} * mu_i
        Eigen::Vector3f normal_acc{Eigen::Vector3f::Zero()};
        float           normal_weight{0.0f};
        // Accumulated shape: merged via parallel-axis formula so the extent grows
        // as the surface is observed from different positions.
        Eigen::Matrix3f shape_acc{Eigen::Matrix3f::Zero()}; // current merged covariance
        Eigen::Vector3f shape_mu{Eigen::Vector3f::Zero()};  // weighted mean of observation positions
        float           shape_weight{0.0f};
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct SurfelMeta {
        uint32_t unobserved_frames{0};
        bool     mature{false};
    };

    struct ActiveSurfel {
        Surfel      estimate;   // current best estimate in submap-local frame
        FusionState fusion;
        SurfelMeta  meta;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct Voxel {
        std::vector<ActiveSurfel> surfels;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    using VoxelMap = std::unordered_map<VoxelKey, Voxel, VoxelKeyHash>;

    // HELPERS - keys/geometry
    VoxelKey to_coarse_key(const Eigen::Vector3f& p_local) const;
    Surfel transform_to_local(const Surfel& s, const Eigen::Isometry3f& T_local_sensor) const;

    // HELPERS - per-frame passes
    void fuse_surfels(const Frame& frame, const Eigen::Isometry3f& T_local_sensor);
    void collect_candidates(const VoxelKey& home, std::vector<ActiveSurfel*>& candidates);
    bool fuse_into(ActiveSurfel& ms, const Surfel& s_local);
    void insert_surfel(const VoxelKey& home, Voxel& voxel, const Surfel& s_local);
    // Recompute position/covariance/normal from FusionState — called after every fusion.
    void recompute_estimate(ActiveSurfel& ms);
    // Compute shape and confidence — deferred to freeze, not needed for gating.
    void finalize_estimate(ActiveSurfel& ms);

    // HELPERS - lifecycle
    void tick_unobserved();
    void evict_immature();

    // State
    Config cfg_;

    Eigen::Isometry3f T_origin_world_;
    Eigen::Isometry3f T_origin_world_inv_;
    Eigen::Isometry3f T_latest_world_;

    VoxelMap voxel_map_;

    size_t total_surfel_count_{0};

    // Rollover metrics
    uint32_t frame_count_{0};
    float    accumulated_translation_{0.0f};
    float    accumulated_rotation_rad_{0.0f};
    int64_t  stamp_ns_start_{0};

    // Scratch buffer for candidate collection — reused each surfel to avoid alloc
    std::vector<ActiveSurfel*> candidate_scratch_;
};


} // namespace smip_uav

#endif
