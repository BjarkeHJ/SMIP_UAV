#include "mapping/map_state_container.hpp"

#include <stdexcept>

namespace smip_uav {

SubmapId MapStateContainer::commit_submap(FrozenSubmap&& submap) {
    std::unique_lock lock(mutex_);

    const SubmapId id = next_id_++;
    submap.id = id;

    assert(id_to_index_.size() == id);
    id_to_index_.push_back(frozen_submaps_.size());

    frozen_submaps_.push_back(std::make_unique<FrozenSubmap>(std::move(submap)));

    return id;
}

std::optional<SubmapId> MapStateContainer::query_overlap(const Eigen::Vector3f& p_world) const {
    std::shared_lock lock(mutex_);

    for (const auto& fs : frozen_submaps_) {
        if (fs->overlaps_point(p_world)) {
            return fs->id;
        }
    }

    return std::nullopt;
}

MapSnapshot MapStateContainer::snapshot() const {
    std::shared_lock lock(mutex_);

    MapSnapshot snap;
    snap.total_submaps = static_cast<uint32_t>(frozen_submaps_.size());
    snap.views.reserve(frozen_submaps_.size());

    for (const auto& fs : frozen_submaps_) {
        MapSnapshot::SubmapView v;
        v.id = fs->id;
        v.T_submap_world = fs->T_submap_world;
        v.centroid_world = fs->T_submap_world * fs->centroid;
        v.radius = fs->radius;
        v.stamp_ns_start = fs->stamp_ns_start;
        v.stamp_ns_end = fs->stamp_ns_end;
        v.surfel_count = static_cast<uint32_t>(fs->surfels.size());
        snap.views.push_back(v);
    }

    return snap;
}

void MapStateContainer::apply_pose_corrections(const std::vector<PoseCorrection>& corrections) {
    std::unique_lock lock(mutex_);
 
    for (const auto& c : corrections) {
        if (c.id >= id_to_index_.size()) continue;
        const size_t idx = id_to_index_[c.id];
        frozen_submaps_[idx]->T_submap_world = c.T_submap_world_corrected;
    }
}

bool MapStateContainer::read_submap(SubmapId id, const std::function<void(const FrozenSubmap&)>& fn) const {
    std::shared_lock lock(mutex_);
 
    if (id >= id_to_index_.size()) return false;
    fn(*frozen_submaps_[id_to_index_[id]]);
    return true;
}

size_t MapStateContainer::num_frozen_submaps() const {
    std::shared_lock lock(mutex_);
    return frozen_submaps_.size();
}

} // smip_uav