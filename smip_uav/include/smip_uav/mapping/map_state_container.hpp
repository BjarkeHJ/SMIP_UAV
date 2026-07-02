#ifndef SMIP_MAP_STATE_CONTAINER_HPP_
#define SMIP_MAP_STATE_CONTAINER_HPP_

#include <vector>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <functional>
#include <optional>

#include "core/types.hpp"
#include "core/frozen_submap.hpp"

namespace smip_uav {

struct PoseCorrection {
    SubmapId id;
    Eigen::Isometry3f T_submap_world_corrected;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct MapSnapshot {
    // lightweight view of each frozen submap - id + current pose + centroid_world
    struct SubmapView {
        SubmapId id;
        Eigen::Isometry3f T_submap_world;
        Eigen::Vector3f centroid_world;
        float radius;
        int64_t stamp_ns_start;
        int64_t stamp_ns_end;
        uint32_t surfel_count;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    std::vector<SubmapView> views; // one per frozen submap
    uint32_t total_submaps;
};

// MapStateContainer owns all frozen submaps. Shared by the ActiveMapNode and PoseOptimizerNode via shared_ptr
// Threading model
// ---------------
//   shared_mutex protects all state.
//   Reads  -> shared_lock   (multiple readers concurrently OK)
//   Writes -> unique_lock   (exclusive; held only for pointer splice / pose
//                            patch — never during heavy computation)
//
// The back-end MUST:
//   1. Call snapshot()         <- shared_lock, very brief
//   2. Run optimisation        <- no lock held
//   3. Call apply_pose_corrections()  <- unique_lock, very brief
//
// The front-end MUST:
//   1. Call commit_submap()    <- unique_lock, very brief (just a push_back)
//   2. Call query_overlap()    <- shared_lock, very brief
class MapStateContainer {
public:
    MapStateContainer() = default;
    MapStateContainer(const MapStateContainer&) = delete;
    MapStateContainer& operator=(const MapStateContainer&) = delete;

    // ActiveMapNode (front-end) interface
    SubmapId commit_submap(FrozenSubmap&& submap);
    std::optional<SubmapId> query_overlap(const Eigen::Vector3f& p_world) const;

    // PoseOptimizerNode (back-end) interface
    MapSnapshot snapshot() const;
    void apply_pose_corrections(const std::vector<PoseCorrection>& corrections);

    // Read a single frozen submap by id, invoking `fn` under a shared lock.
    // Use this when the back-end needs surfel-level access (e.g. ICP).
    // `fn` must not call back into MapStateContainer (deadlock).
    //
    // Example:
    //   container->read_submap(id, [&](const FrozenSubmap& fs) {
    //       run_icp(fs.surfels, query_cloud);
    //   });
    //
    // Returns false if the id is not found.
    bool read_submap(SubmapId id, const std::function<void(const FrozenSubmap&)>& fn) const; 

    // general interface (shared)
    size_t num_frozen_submaps() const;

    // Latest map<-odom correction from the back-end (GlobalMapNode).
    // Kept under its own lock since it is updated/read independently of the submap state.
    void write_map_odom(const Eigen::Isometry3f& T_map_odom);
    Eigen::Isometry3f read_map_odom() const;

private:
    mutable std::shared_mutex mutex_;
    std::vector<std::unique_ptr<FrozenSubmap>> frozen_submaps_;
    SubmapId next_id_{0};
    std::vector<size_t> id_to_index_;

    mutable std::shared_mutex map_odom_mutex_;
    Eigen::Isometry3f latest_map_odom_tf_{Eigen::Isometry3f::Identity()};
};
    
} // namespace smip_uav


#endif
