#ifndef SMIP_FROZEN_SUBMAP_HPP_
#define SMIP_FROZEN_SUBMAP_HPP_

#include <vector>
#include <memory>
#include <cstdint>
#include <Eigen/Geometry>
#include <nanoflann.hpp>

#include "core/types.hpp"
#include "core/surfel.hpp"

namespace smip_uav {

struct SurfelCloud {
    const std::vector<Surfel>& pts;
    explicit SurfelCloud(const std::vector<Surfel>& v) : pts(v) {}

    inline size_t kdtree_get_point_count() const { return pts.size(); }
    inline float kdtree_get_pt(size_t idx, size_t dim) const { return pts[idx].position(static_cast<int>(dim)); }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
};

// KD-tree type alias — 3D, float, L2 metric, uint32_t index
using SurfelKDTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, SurfelCloud>, SurfelCloud, 3, uint32_t>;

struct FrozenSubmap {
    SubmapId id{kInvalidSubmapId};

    Eigen::Isometry3f T_submap_world{Eigen::Isometry3f::Identity()}; // only writeable field during graph opt...
    Eigen::Isometry3f T_submap_world_origin{Eigen::Isometry3f::Identity()}; // set at freeze-time

    std::vector<Surfel> surfels; // Should i have both this AND the kdtree?

    Eigen::Vector3f centroid{Eigen::Vector3f::Zero()};
    float radius{0.0f};

    std::unique_ptr<SurfelCloud> kdtree_cloud_;
    std::unique_ptr<SurfelKDTree> kdtree_;

    int64_t stamp_ns_start{0};
    int64_t stamp_ns_end{0};
    uint32_t frame_count{0};
    float accumulated_translation{0.0f};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Spatial queries
    bool overlaps_point(const Eigen::Vector3f& p_world) const {
        const Eigen::Vector3f p_local = T_submap_world.inverse() * p_world;
        return (p_local - centroid).squaredNorm() < radius * radius;
    }

    size_t knn_search(const Eigen::Vector3f& query_local, size_t k, std::vector<uint32_t>& out_indices, std::vector<float>& out_dist_sq) const {
        if (!kdtree_ || surfels.empty()) return 0;
        out_indices.resize(k);
        out_dist_sq.resize(k);
        nanoflann::KNNResultSet<float, uint32_t> rs(k);
        rs.init(out_indices.data(), out_dist_sq.data());
        kdtree_->findNeighbors(rs, query_local.data(), nanoflann::SearchParams());
        const size_t found = rs.size();
        out_indices.resize(found);
        out_dist_sq.resize(found);
        return found;
    }

    size_t radius_search(const Eigen::Vector3f& query_local, float radius_m, std::vector<std::pair<uint32_t, float>>& out) const {
        if (!kdtree_ || surfels.empty()) return 0;
        out.clear();
        const float r_sq = radius_m * radius_m;
        return kdtree_->radiusSearch(query_local.data(), r_sq, out, nanoflann::SearchParams());
    }

    void compute_bounds() {
        if (surfels.empty()) {
            centroid.setZero();
            radius = 0.0f;
            return;
        }

        centroid.setZero();
        for (const auto& s : surfels) {
            centroid += s.position;
        }
        centroid /= static_cast<float>(surfels.size());
        
        radius = 0.0f;
        for (const auto& s : surfels)
            radius = std::max(radius, (s.position - centroid).norm());
    }
 
    void build_kdtree() {
        if (surfels.empty()) return;
        kdtree_cloud_ = std::make_unique<SurfelCloud>(surfels);
        kdtree_ = std::make_unique<SurfelKDTree>(
            3, *kdtree_cloud_,
            nanoflann::KDTreeSingleIndexAdaptorParams(10 /*max_leaf_size*/)
        );
    }
};


} // namespace smip_uav

#endif