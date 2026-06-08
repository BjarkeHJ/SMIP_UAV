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
        float subvoxel_size{0.05f};

        float min_normal_dot{0.75f};
        float max_point_to_plane_m{0.15f};
        float max_mahalanobis_sq{9.0f};

        uint32_t maturity_obs_count{10};
        uint32_t max_unobserved_frames{8};

        uint32_t min_bins_for_refit{8};
        float omega_regularization{1e-6f};

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
    size_t bin_point_count() const { return total_bin_count_; }

    const Eigen::Isometry3f& T_origin_world() const { return T_origin_world_; }
    const Eigen::Isometry3f& T_latest_world() const { return T_latest_world_; }

    // Iterate all accumulated bin points in submap-local frame.
    // Fn: void(const PointBin&)
    template<typename Fn>
    void for_each_bin_point(Fn&& fn) const {
        for (const auto& [key, voxel] : voxel_map_) {
            for (const auto& [sk, bin] : voxel.point_bins) {
                fn(bin.position, bin.normal, bin.weight, bin.count);
            }
        }
    }
    
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

    struct SubKey {
        int16_t x, y, z;
        bool operator==(const SubKey& o) const {
            return x == o.x && y == o.y && z == o.z;
        }
    };

    struct SubKeyHash {
        size_t operator()(const SubKey& k) const noexcept {
            size_t h = 2166136261u;
            h ^= static_cast<size_t>(static_cast<uint16_t>(k.x)); h *= 16777619u;
            h ^= static_cast<size_t>(static_cast<uint16_t>(k.y)); h *= 16777619u;
            h ^= static_cast<size_t>(static_cast<uint16_t>(k.z)); h *= 16777619u;
            return h;
        }
    };

    struct PointBin {
        Eigen::Vector3f position{Eigen::Vector3f::Zero()};
        Eigen::Vector3f normal{Eigen::Vector3f::Zero()};
        float           weight{0.0f};
        uint32_t        count{0};
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    using SubvoxelGrid = std::unordered_map<SubKey, PointBin, SubKeyHash>;

    struct FusionState {
        Eigen::Matrix3f Omega{Eigen::Matrix3f::Zero()}; // sum R_i^{-1}
        Eigen::Vector3f xi{Eigen::Vector3f::Zero()};    // sum R_i^{-1} * mu_i
        Eigen::Vector3f normal_acc{Eigen::Vector3f::Zero()};
        float           normal_weight{0.0f};
        Eigen::Matrix3f shape_acc{Eigen::Matrix3f::Zero()};
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
        // Multiple active surfels per voxel — geometric content determines identity
        std::vector<ActiveSurfel> surfels;
 
        // Dense point geometry accumulated from all frames touching this voxel.
        // Keys are relative to this voxel's origin corner in subvoxel units.
        SubvoxelGrid point_bins;
 
        // World-local position of this voxel's (0,0,0) corner.
        // Set on first insertion, used to compute subvoxel relative keys.
        Eigen::Vector3f origin{Eigen::Vector3f::Zero()};
        bool origin_set{false};
 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    using VoxelMap = std::unordered_map<VoxelKey, Voxel, VoxelKeyHash>;

    // HELPERS - keys/geometry
    // Compute coarse voxel key for a submap-local position
    VoxelKey to_coarse_key(const Eigen::Vector3f& p_local) const;
    // Compute subvoxel key relative to voxel origin
    SubKey to_sub_key(const Eigen::Vector3f& p_local, const Eigen::Vector3f& voxel_origin) const;
    // Transform a frame surfel from sensor frame to submap-local frame
    Surfel transform_to_local(const Surfel& s, const Eigen::Isometry3f& T_local_sensor) const;

    // HELPERS - per-frame passes
    // Pass 1: insert valid pixels into subvoxel grids
    void insert_pixels(const Frame& frame, const Eigen::Isometry3f& T_local_sensor);
    // Pass 2: fuse frame surfels into active surfels
    void fuse_surfels(const Frame& frame, const Eigen::Isometry3f& T_local_sensor);
    // Collect all candidate ActiveSurfel pointers from 27-cell neighbourhood
    void collect_candidates(const VoxelKey& home, std::vector<ActiveSurfel*>& candidates);
    // Attempt fusion of s_local into an existing ActiveSurfel.
    // Returns true if gating passed and fusion occurred.
    bool fuse_into(ActiveSurfel& ms, const Surfel& s_local);
    // Insert s_local as a new ActiveSurfel in voxel at home key
    void insert_surfel(const VoxelKey& home, Voxel& voxel,const Surfel& s_local);
    // Recompute Surfel estimate from FusionState after an update
    void recompute_estimate(ActiveSurfel& ms);

    // HELPERS - lifecycle
    // Tick unobserved counters on all immature surfels; evict if exceeded.
    // Called at the start of each register_frame().
    void tick_unobserved();
    // Freeze-time: evict immature surfels from all voxels.
    void evict_immature();

    // State
    Config cfg_;
 
    Eigen::Isometry3f T_origin_world_;
    Eigen::Isometry3f T_origin_world_inv_;
    Eigen::Isometry3f T_latest_world_;
 
    VoxelMap voxel_map_;
 
    // Maintained for O(1) surfel_count() / bin_point_count() without iterating all voxels
    size_t total_surfel_count_{0};
    size_t total_bin_count_{0};
 
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