#ifndef SURFEL_HPP_
#define SURFEL_HPP_

#include <cstdint>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "common/point_types.hpp"

namespace smip_uav {

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
    struct Config {
        
    };

    // Fixed constants (non configurable)
    static constexpr unsigned int MAX_NUM_SURFELS = 6;
    static constexpr unsigned int MIN_SURFEL_POINTS = 10;
    static constexpr unsigned int MAX_SURFEL_POINTS = 10000;

    Surfel();
    explicit Surfel(const Config&);

    void update(const std::vector<PointNormal>& pns);

    // Accessors
    const Eigen::Vector3f& mean() const { return mean_; }
    const Eigen::Vector3f& normal() const { return normal_; }
    const Eigen::Matrix3f& covariance() const { return covariance_; }
    const Eigen::Vector3f& eigen_values() const { return eigen_values_; }
    const Eigen::Matrix3f& eigen_vectors() const { return eigen_vectors_; }
    double ts_create() const { return ts_create_; }
    double ts_update() const { return ts_update_ ; }
    size_t point_count() const { return count_; }
    bool is_valid() const { return valid_; }

private:
    void compute_eigen_decomp();
    
    size_t count_{0};

    Eigen::Vector3f mean_{Eigen::Vector3f::Zero()};
    Eigen::Vector3f normal_{Eigen::Vector3f::Zero()};
    
    Eigen::Matrix3f covariance_{Eigen::Matrix3f::Identity()};
    Eigen::Matrix3f eigen_vectors_{Eigen::Matrix3f::Identity()};
    Eigen::Vector3f eigen_values_{Eigen::Vector3f::Zero()};

    Eigen::Vector3f avg_view_dir_{Eigen::Vector3f::Zero()};

    VoxelKey key_;

    double ts_create_{0};
    double ts_update_{0};

    bool valid_{false};
};

} // smip_uav


#endif