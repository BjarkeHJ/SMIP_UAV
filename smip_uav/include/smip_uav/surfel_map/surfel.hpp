#ifndef SURFEL_HPP_
#define SURFEL_HPP_

#include <cstdint>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include "common/point_types.hpp"

namespace smip_uav {

inline constexpr float NS_TO_SEC = 1e-9f;
inline constexpr float SEC_TO_NS = 1e9f;

struct VoxelKey {
    int32_t x,y,z;
    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    bool operator!=(const VoxelKey& other) const {
        return !(*this == other);
    }

    bool operator<(const VoxelKey& other) const {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z; 
    }
};

struct VoxelKeyHash {
    size_t operator()(const VoxelKey& k) const {
        // spread bits of each coordinate then interleave
        auto spread = [](uint32_t v) -> uint64_t {
            uint64_t x = v;
            x = (x | x << 32) & 0x1f00000000ffff;
            x = (x | x << 16) & 0x1f0000ff0000ff;
            x = (x | x << 8)  & 0x100f00f00f00f00f;
            x = (x | x << 4)  & 0x10c30c30c30c30c3;
            x = (x | x << 2)  & 0x1249249249249249;
            return x;
        };
        return spread(k.x) | (spread(k.y) << 1) | (spread(k.z) << 2);
    }
};

class Surfel {
public:
    Surfel() = default;

    void accumulate(const PointNormal& pn, const Eigen::Vector3f& view_dir, int64_t timestamp); // Add a single weighted point
    void merge_from(const Surfel& other); // Absorb existing surfel from same voxel
    void merge_from_translated(const Surfel& other, const Eigen::Vector3f& delta_c); // Absorb existing surfel from different voxel

    void recompute();

    // Accessors
    const Eigen::Vector3f& mean() const { return mean_; }
    const Eigen::Vector3f& normal() const { return normal_; }
    const Eigen::Matrix3f& covariance() const { return covariance_; }
    const Eigen::Matrix3f& eigenvectors() const { return eigenvectors_; }
    const Eigen::Vector3f& eigenvalues() const { return eigenvalues_; }

    Eigen::Vector3f world_mean(float voxel_size) const; // reconstruct world-frame position: voxel_center + mean_

    float planarity() const;
    float linearity() const;
    float sphericity() const;
    float priority(int64_t now, size_t min_points_mature) const;
    
    size_t point_count() const { return count_; }
    float total_weight() const  { return W_; }
    int64_t created_at() const { return ts_create_; }
    int64_t updated_at() const { return ts_update_ ; }
    bool needs_recompute() const { return dirty_; }
    const VoxelKey& voxel_key() const { return key_; }

    bool is_mature(size_t min_points) const { return count_ >= min_points; }
    uint32_t id() const { return id_; }

    // Surfel Initialization
    void seed(const PointNormal& pn, const Eigen::Vector3f& view_dir, const VoxelKey& key, int64_t timestamp);

private:
    // Sufficient statistics
    float W_{0.0f};
    Eigen::Vector3f S1_{Eigen::Vector3f::Zero()};
    Eigen::Matrix3f S2_{Eigen::Matrix3f::Zero()};
    size_t count_{0};

    Eigen::Vector3f sum_view_dir_{Eigen::Vector3f::Zero()};
    
    Eigen::Vector3f mean_{Eigen::Vector3f::Zero()};
    Eigen::Vector3f normal_{Eigen::Vector3f::Zero()};
    Eigen::Matrix3f covariance_{Eigen::Matrix3f::Identity()};
    Eigen::Matrix3f eigenvectors_{Eigen::Matrix3f::Identity()};
    Eigen::Vector3f eigenvalues_{Eigen::Vector3f::Zero()};

    VoxelKey key_{0,0,0};
    uint32_t id_{0};

    int64_t ts_create_{0};
    int64_t ts_update_{0};

    bool dirty_{true};
};

} // smip_uav


#endif