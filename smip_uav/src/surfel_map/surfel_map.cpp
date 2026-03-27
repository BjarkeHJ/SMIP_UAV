#include "surfel_map/surfel_map.hpp"

namespace smip_uav {

SurfelMap::SurfelMap(const Config& cfg) : grid_(cfg.grid_config), config_(cfg) {}

void SurfelMap::integrate_frame(const Frame& frame) {
    const Eigen::Isometry3f& T_ws = frame.tf_pose;
    const Eigen::Matrix3f R_ws = T_ws.rotation();
    const Eigen::Vector3f sensor_origin_world = T_ws.translation();

    for (size_t i = 0; i < frame.pixels.size(); ++i) {
        const PointNormal& pn  = frame.pixels[i];
        if (!Frame::is_valid(pn)) continue;

        const Eigen::Vector3f p_sensor(pn.px, pn.py, pn.pz);
        const Eigen::Vector3f n_sensor(pn.nx, pn.ny, pn.nz);
        const Eigen::Vector3f p_world = T_ws * p_sensor;
        const Eigen::Vector3f n_world = (R_ws * n_sensor).normalized();

        Eigen::Vector3f view_dir = sensor_origin_world - p_world;
        const float len = view_dir.norm();
        if (len < 1e-6f) continue;
        view_dir /= len;
        
        PointNormal pn_tf = {p_world.x(), p_world.y(), p_world.z(), n_world.x(), n_world.y(), n_world.z(), pn.w};
        associate_and_fuse(pn_tf, view_dir, frame.timestamp);
    }

    frame_count_++;

    if (frame_count_ % config_.maintanence_interval_N == 0) {
        run_maintenance(frame.timestamp);
    }
}

void SurfelMap::associate_and_fuse(const PointNormal& pn, const Eigen::Vector3f& view_dir, int64_t timestamp) {
    const VoxelKey key = grid_.to_key(pn.px, pn.py, pn.pz);
    Voxel& voxel = grid_.get_or_create(key);

    // Voxel-local position
    const Eigen::Vector3f center = grid_.voxel_center(key);
    PointNormal pn_voxel = {pn.px-center[0], pn.py-center[1], pn.pz-center[2], pn.nx, pn.ny, pn.nz, pn.w};
    
    // Find a suitable surfel -> nullptr if unable
    Surfel* match = find_best_match(voxel, pn_voxel);

    if (match) {
        if (match->point_count() < config_.surfel_max_points) {
            match->accumulate(pn_voxel, view_dir, timestamp);
        }
    } else {
        handle_new_surface(voxel, pn_voxel, view_dir, key, timestamp);
    }
}

Surfel* SurfelMap::find_best_match(Voxel& voxel, const PointNormal& pn) {
    if (voxel.empty()) return nullptr;
    
    const Eigen::Vector3f p{pn.px, pn.py, pn.pz};
    const Eigen::Vector3f n{pn.nx, pn.ny, pn.nz};

    Surfel* best = nullptr;
    float best_score = std::numeric_limits<float>::max();

    for (auto& surfel : voxel) {
        const float normal_thresh = surfel.is_mature(config_.surfel_min_points) ? config_.assoc_normal_cos_mature : config_.assoc_normal_cos_tentative;

        // Normal vector similarity
        const float n_dot = std::abs(surfel.normal().dot(n));
        if (n_dot < normal_thresh) continue;

        // Surfel-Normal-Distance displacement (point-to-plane)
        const float d_perp = std::abs(surfel.normal().dot(p - surfel.mean()));
        if (d_perp > config_.assoc_point_to_plane_max) continue;

        const float score = d_perp / n_dot;
        if (score < best_score) {
            best_score = score;
            best = &surfel;
        }
    }

    return best;
}

void SurfelMap::handle_new_surface(Voxel& voxel, const PointNormal& pn, const Eigen::Vector3f& view_dir, const VoxelKey& key, int64_t timestamp) {
    const Eigen::Vector3f n{pn.nx, pn.ny, pn.nz};
    
    for (const auto& surfel : voxel) {
        const float n_dot = std::abs(surfel.normal().dot(n));
        if (n_dot > config_.creation_min_normal_separaton_cos) return; // too similar - drop point
    }

    if (voxel.full()) {
        if (!try_free_slot(voxel, key, timestamp)) {
            return; // voxel at capacity with good surfels
        }
    }

    Surfel s;
    s.seed(pn, view_dir, key, timestamp);
    voxel.try_add(s);
}

bool SurfelMap::try_free_slot(Voxel& voxel, const VoxelKey& key, int64_t timestamp) {
    // Attempt 1: Merge compatible pair
    for (uint8_t i = 0; i < voxel.count; ++i) {
        for (uint8_t j = i + 1; j < voxel.count; ++j) {
            voxel.surfels[i].recompute();
            voxel.surfels[j].recompute();

            if (merge_ok_same_voxel(voxel.surfels[i], voxel.surfels[j], key)) {
                if (voxel.surfels[i].point_count() >= voxel.surfels[j].point_count()) {
                    voxel.surfels[i].merge_from(voxel.surfels[j]);
                    voxel.remove_at(j);
                } else {
                    voxel.surfels[j].merge_from(voxel.surfels[i]);
                    voxel.remove_at(i);
                }
                return true;
            }
        }
    }

    // Attemp 2: Evict weaket tentative
    float worst_priority = std::numeric_limits<float>::max();
    uint8_t worst_idx = 0;
    bool found = false;

    for (uint8_t i = 0; i < voxel.count; ++i) {
        const auto& s = voxel.surfels[i];
        if (!s.is_mature(config_.surfel_min_points)) {
            const float p = s.priority(timestamp, config_.surfel_min_points);
            if (p < worst_priority) {
                worst_priority = p;
                worst_idx = i;
                found = true;
            }
        }
    }

    if (found) {
        voxel.remove_at(worst_idx);
        return true;
    }

    return false; // could not free a slot in voxel for new surfel
}

void SurfelMap::run_maintenance(int64_t timestamp) {
    std::vector<VoxelKey> keys;
    keys.reserve(grid_.size());
    for (auto& [key, voxel] : grid_) {
        keys.push_back(key);
    }

    // Recompute all dirty surfels
    for (auto& key : keys) {
        if (Voxel* v = grid_.get(key)) {
            recompute_dirty(*v);
        }
    }

    // Intra-voxel merge of similar surfels
    for (auto& key : keys) {
        if (Voxel* v = grid_.get(key)) {
            merge_similar(*v);
        }
    }

    // Cross boundary merge
    for (auto& key : keys) {
        if (Voxel* v = grid_.get(key)) {
            merge_boundary_surfels(key, *v);
        }
    }

    // Evict stale tentatives
    for (auto& key : keys) {
        if (Voxel* v = grid_.get(key)) {
            evict_stale(*v, timestamp);
        }
    }

    // Evict low quality mature surfels
    for (auto& key : keys) {
        if (Voxel* v = grid_.get(key)) {
            evict_low_quality(*v);
        }
    }

    // Remove empty voxels 
    for (auto& key : keys) {
        if (const Voxel* v = grid_.get(key)) {
            if (v->empty()) grid_.remove(key);
        }
    }

    // Snapshot the maintained map and make available for "public"
    update_public();
}

void SurfelMap::recompute_dirty(Voxel& voxel) {
    for (auto& surfel : voxel) {
        if (surfel.needs_recompute()) surfel.recompute();
    }
}

void SurfelMap::merge_similar(Voxel& voxel) {
    if (voxel.count < 2) return;

    const VoxelKey& key = voxel.surfels[0].voxel_key();

    bool merged = true;
    while (merged) {
        merged = false;
        for (uint8_t i = 0; i < voxel.count && !merged; ++i) {
            for (uint8_t j = i + 1; j < voxel.count && !merged; ++j) {
                if (merge_ok_same_voxel(voxel.surfels[i], voxel.surfels[j], key)) {
                    if (voxel.surfels[i].point_count() >= voxel.surfels[j].point_count()) {
                        voxel.surfels[i].merge_from(voxel.surfels[j]);
                        voxel.surfels[i].recompute();
                        voxel.remove_at(j);
                    } else {
                        voxel.surfels[j].merge_from(voxel.surfels[i]);
                        voxel.surfels[j].recompute();
                        voxel.remove_at(i);
                    }
                    merged = true;
                }
            }
        }
    }
}

void SurfelMap::merge_boundary_surfels(const VoxelKey& key, Voxel& voxel) {
    const float vs = grid_.voxel_size();
    const float margin = vs * config_.boundary_margin_ratio;
    const Eigen::Vector3f center = grid_.voxel_center(key);
    const float half = vs * 0.5f;

    for (uint8_t i = 0; i < voxel.count; ++i) {
        const Eigen::Vector3f mean_local = voxel.surfels[i].mean();
        const Eigen::Vector3f mean_world = center + mean_local;

        for (int axis = 0; axis < 3; ++axis) {
            const float dist_to_face = half - std::abs(mean_local(axis));
            if (dist_to_face > margin) continue; // not close enough to voxel boundary

            const int32_t dir = (mean_local(axis) > 0.0f) ? 1 : -1;
            VoxelKey nkey = key;
            (&nkey.x)[axis] += dir; // traverse grid in one axis and get neighbor key

            Voxel* neighbor = grid_.get(nkey);
            if (!neighbor) continue;

            const Eigen::Vector3f n_center = grid_.voxel_center(nkey);

            for (uint8_t j = 0; j < neighbor->count; ++j) {
                const Eigen::Vector3f n_mean_world = n_center + neighbor->surfels[j].mean();
                
                if (!merge_ok(voxel.surfels[i], mean_world, neighbor->surfels[j], n_mean_world)) {
                    continue; 
                }

                // merge smaller into larger (maybe quality instead???)
                const bool i_is_larger = voxel.surfels[i].point_count() >= neighbor->surfels[j].point_count();

                if (i_is_larger) {
                    const Eigen::Vector3f delta_c = n_center - center;
                    voxel.surfels[i].merge_from_translated(neighbor->surfels[j], delta_c);
                    voxel.surfels[i].recompute();
                    neighbor->remove_at(j);
                } else {
                    const Eigen::Vector3f delta_c = center - n_center;
                    neighbor->surfels[j].merge_from_translated(voxel.surfels[i], delta_c);
                    neighbor->surfels[j].recompute();
                    voxel.remove_at(i);
                    
                    // Index shifted - restart outer loop
                    i = static_cast<uint8_t>(-1); // becomes 0 after ++i
                    break;
                }
                break; // re-check updated surfel against remaining neighbors
            }

            if (i == static_cast<uint8_t>(-1)) break;
        }
    }
}

void SurfelMap::evict_stale(Voxel& voxel, int64_t timestamp) {
    for (uint8_t i = 0; i < voxel.count; ) {
        const auto& s = voxel.surfels[i];
        if (!s.is_mature(config_.surfel_min_points)) {
            const float age = static_cast<float>(timestamp - s.updated_at());
            if (age > config_.tentative_timeout_sec) {
                voxel.remove_at(i);
                continue;
            }
        }
        ++i;
    }
}

void SurfelMap::evict_low_quality(Voxel& voxel) {
    for (uint8_t i = 0; i < voxel.count; ) {
        const auto& s = voxel.surfels[i];
        if (s.is_mature(config_.surfel_min_points) && s.planarity() < config_.min_planarity) {
            voxel.remove_at(i);
            continue;
        }
        ++i;
    }
}

bool SurfelMap::merge_ok(const Surfel& a, const Eigen::Vector3f& pos_a_world, const Surfel& b, const Eigen::Vector3f& pos_b_world) const {
    const float n_dot = std::abs(a.normal().dot(b.normal()));
    if (n_dot < config_.merge_normal_cos) return false;

    // Symmetric point-to-place: each mean must close to the others plane
    const Eigen::Vector3f delta = pos_b_world - pos_a_world;
    const float d_a = std::abs(a.normal().dot(delta));
    const float d_b = std::abs(b.normal().dot(delta));

    return std::max(d_a, d_b) <= config_.merge_point_to_plane_max;
}

bool SurfelMap::merge_ok_same_voxel(const Surfel& a, const Surfel& b, const VoxelKey& key) const {
    const Eigen::Vector3f center = grid_.voxel_center(key);
    return merge_ok(a, center + a.mean(), b, center + b.mean());
}

void SurfelMap::update_public() {
    grid_public_ = grid_;
    has_public_ = true;
}

std::vector<const Surfel*> SurfelMap::surfels() const {
    std::vector<const Surfel*> out;
    if (!has_public_) return out;

    for (const auto& [key, voxel] : grid_public_) {
        for (const auto& surfel : voxel) {
            if (surfel.is_mature(config_.surfel_min_points) && surfel.planarity() >= config_.min_planarity) {
                out.push_back(&surfel);
            }
        }
    }

    return out;
}

} // smip_uav