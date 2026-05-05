#include "surfel_map/surfel_map.hpp"

namespace smip_uav {
    
SurfelMap::SurfelMap(const Config& cfg) : cfg_(cfg) {
    grid_ = std::make_unique<VoxelGrid>(cfg_.grid_config);

    log_2pi_1_5_ = 1.5f * std::log(2.0f * static_cast<float>(M_PI));
    const float vs = cfg_.grid_config.voxel_size;
    const float V_voxel = vs * vs * vs;
    log_lambda_new_ = std::log(cfg_.pi_spawn) - std::log(V_voxel);

    const float sigma_n = cfg_.normal_sigma;
    inv_2_sigma_n_sq_ = 1.0f / (2.0f * sigma_n * sigma_n);
    merge_normal_cos_ = std::cos(cfg_.merge_normal_k * sigma_n);
}



void SurfelMap::update_map(const std::vector<FrameSurfel>& frame_surfels, const Eigen::Isometry3f& pose, int64_t timestamp_ns) {
    if (frame_surfels.empty()) return;

    integrate(frame_surfels, pose, timestamp_ns);

    frame_count_++;
    if (cfg_.merge_interval > 0 && (frame_count_ % cfg_.merge_interval) == 0) {
        merge();
    }
}

void SurfelMap::integrate(const std::vector<FrameSurfel>& frame_surfels, const Eigen::Isometry3f& pose, int64_t timestamp_ns) {
    // Clear buffers
    accums_.clear();
    spawn_candidates_.clear();
    resp_.clear();

    // E-Step: Compute responsibilities and accumulate
    for (const FrameSurfel& fs : frame_surfels) {
        FrameSurfel fs_w = transform_surfel_to_world(fs, pose);

        resp_.clear();
        const float r_new = compute_responsibilities(fs_w, resp_);
        const float w_k = fs_w.weight * fs_w.view_cos_theta;

        // accumulate weighted observations into each responsible component
        for (const auto& entry : resp_) {
            if (entry.r < 1e-6f) continue;

            auto& acc = accums_[entry.component];
            const float wr = w_k * entry.r;
            acc.delta_W += wr;
            acc.delta_S1 += wr * fs_w.centroid;
            acc.delta_S2 += wr * (fs_w.centroid * fs_w.centroid.transpose() + fs_w.C_shape);
        }

        if (r_new > cfg_.spawn_residual) {
            spawn_candidates_.push_back(fs_w);
        }
    }

    // M-Step: Apply accumulated deltas, reconstruct params
    const float gamma = cfg_.gamma_forget;
    for (auto& [ms_ptr, acc] : accums_) {
        ms_ptr->W = gamma * ms_ptr->W + acc.delta_W;
        ms_ptr->S1 = gamma * ms_ptr->S1 + acc.delta_S1;
        ms_ptr->S2 = gamma * ms_ptr->S2 + acc.delta_S2;
        ms_ptr->reconstruct();
        ms_ptr->obs_count++;
        ms_ptr->last_seen = timestamp_ns;

        // Delta update tracking
        updated_ids_.insert(ms_ptr->id);
    }

    // Spawn new
    for (const FrameSurfel& fs_w : spawn_candidates_) {
        spawn(fs_w, timestamp_ns);
    }

    cache_dirty_ = true;
}

float SurfelMap::compute_responsibilities(const FrameSurfel& fs_w, std::vector<RespEntry>& resp_out) {
    const VoxelKey key = grid_->to_key(fs_w.centroid);

    // collect candidates from center voxel + 6-conn-nbs (modifies resp_out)
    auto search_voxel = [&](Voxel& voxel) {
        for (uint8_t i = 0; i < voxel.count; ++i) {
            MapSurfel& ms = voxel.surfels[i];

            // Soft normal penalty: log N(theta; 0, sigma_n) via small-angle approx theta^2 ~ 2(1-cos)
            // Avoids acos in hot path; inv_2_sigma_n_sq_ = 1/(2*sigma_n^2), shared with merge threshold.
            const float dot_n = fs_w.normal.dot(ms.normal);
            const float log_p_normal = -(1.0f - dot_n) * inv_2_sigma_n_sq_;

            const Eigen::Matrix3f M = ms.sigma + fs_w.R;
            const Eigen::LDLT<Eigen::Matrix3f> M_ldlt(M);

            const Eigen::Vector3f nu = fs_w.centroid - ms.mu;
            const float epsilon = nu.dot(M_ldlt.solve(nu)); // nu * M^-1 * nu.T

            // log|M| from LDLT diag
            const float log_det_M = M_ldlt.vectorD().array().abs().log().sum();

            // log(r~)
            const float log_r_tilde = std::log(ms.W + 1e-10f) - log_2pi_1_5_ - 0.5f*log_det_M - 0.5f*epsilon + log_p_normal;

            resp_out.push_back({&ms, log_r_tilde, 0.0f});
        }
    };

    if (Voxel* v = grid_->get(key)) search_voxel(*v); // search center voxel
    grid_->for_each_nb6(key, [&](const VoxelKey&, Voxel& v) { search_voxel(v); }); // search nb-6 voxels
    
    if (resp_out.empty()) {
        return 1.0f; // no candidates at all - entire resp goes to spawn
    }

    // Normalize component log-priors by local neighbourhood total weight so that
    // log(W_j) -> log(W_j / sum_W), keeping the fixed spawn hypothesis calibrated
    // as the map matures and individual W_j values grow without bound.
    float sum_W = 0.0f;
    for (const auto& e : resp_out) sum_W += e.component->W;
    const float log_sum_W = std::log(sum_W + 1e-10f);
    for (auto& e : resp_out) e.log_r_tilde -= log_sum_W;

    // find max log value for numerical stability
    float max_log = log_lambda_new_;
    for (const auto& e : resp_out) {
        max_log = std::max(max_log, e.log_r_tilde);
    }

    // exponentiate and sum
    float sum = std::exp(log_lambda_new_ - max_log); // spawn hypothesis
    for (auto& e : resp_out) {
        e.r = std::exp(e.log_r_tilde - max_log);
        sum += e.r;
    }

    // normalize
    const float inv_sum = 1.0f / (sum + 1e-10f);
    for (auto& e : resp_out) {
        e.r *= inv_sum;
    }

    const float r_new = std::exp(log_lambda_new_ - max_log) * inv_sum;
    return r_new;
}

void SurfelMap::spawn(const FrameSurfel& fs_w, int64_t timestamp_ns) {
    const float w = fs_w.weight;

    MapSurfel ms;
    ms.id = next_id_++;
    ms.W = w;
    ms.S1 = w * fs_w.centroid;
    ms.S2 = w * (fs_w.centroid * fs_w.centroid.transpose() + fs_w.C_shape);
    ms.mu = fs_w.centroid;
    ms.sigma = fs_w.C_shape;
    ms.normal = fs_w.normal;
    ms.obs_count = 1;
    ms.last_seen = timestamp_ns;

    const VoxelKey key = grid_->to_key(ms.mu);
    Voxel& voxel = grid_->get_or_create(key);

    if (voxel.full()) {
        uint8_t min_idx = 0;
        for (uint8_t i = 1; i < voxel.count; ++i) {
            if (voxel.surfels[i].W < voxel.surfels[min_idx].W) min_idx = i;
        }
        if (ms.W <= voxel.surfels[min_idx].W) return; // newcomer weaker than all - drop

        const uint32_t evicted_id = voxel.surfels[min_idx].id;
        voxel.remove_at(min_idx);
        deleted_ids_.insert(evicted_id);
        updated_ids_.erase(evicted_id);
    }

    if (voxel.try_add(ms)) {
        updated_ids_.insert(ms.id);
    }
}


void SurfelMap::merge() {
    struct MergePair {
        MapSurfel* survivor;
        VoxelKey victim_key;
        uint8_t victim_idx;
    };

    std::vector<MergePair> pairs;

    for (auto& [key_a, voxel_a] : *grid_) {

        // Intra-voxel pairs
        for (uint8_t i = 0; i < voxel_a.count; ++i) {
            for (uint8_t j = i + 1; j < voxel_a.count; ++j) {
                MapSurfel& a = voxel_a.surfels[i];
                MapSurfel& b = voxel_a.surfels[j];
            
                if (a.normal.dot(b.normal) < merge_normal_cos_) continue;

                const Eigen::Vector3f d = a.mu - b.mu;
                const Eigen::Matrix3f S = a.sigma + b.sigma;
                const float d2 = d.dot(S.ldlt().solve(d));
                if (d2 >= cfg_.merge_mahal_sq) continue;

                if (a.W >= b.W) {
                    pairs.push_back({&a, key_a, j});
                }
                else {
                    pairs.push_back({&b, key_a, i});
                }
            }
        }

        // Cross-voxel pairs (center vs each nb6)
        grid_->for_each_nb6(key_a, [&](const VoxelKey& key_b, Voxel& voxel_b) {
            // only process if key_a < key_b to avoid doubles
            if (!(key_a < key_b)) return;

            for (uint8_t i = 0; i < voxel_a.count; ++i) {
                for (uint8_t j = 0; j < voxel_b.count; ++j) {
                    MapSurfel& a = voxel_a.surfels[i];
                    MapSurfel& b = voxel_b.surfels[j];

                    if (a.normal.dot(b.normal) < merge_normal_cos_) continue;

                    const Eigen::Vector3f d = a.mu - b.mu;
                    const Eigen::Matrix3f S = a.sigma + b.sigma;
                    const float d2 = d.dot(S.ldlt().solve(d));
                    if (d2 >= cfg_.merge_mahal_sq) continue;

                    if (a.W >= b.W) {
                        pairs.push_back({&a, key_b, j});
                    }
                    else {
                        pairs.push_back({&b, key_a, i});
                    }
                }
            }
        });
    }

    // Execute merges: sort by key (descending index) so removal order is stable
    std::sort(pairs.begin(), pairs.end(), [](const MergePair& a, const MergePair& b) {
        if (a.victim_key != b.victim_key) return a.victim_key < b.victim_key;
        return a.victim_idx > b.victim_idx; // descending: remove from back first
    });

    // Track which ones have alreade been removed (by id) to avoid double-removal
    std::unordered_set<uint32_t> removed_ids;

    for (const auto& mp : pairs) {
        Voxel* victim_voxel = grid_->get(mp.victim_key);
        if (!victim_voxel) continue;
        if (mp.victim_idx >= victim_voxel->count) continue;

        MapSurfel& victim = victim_voxel->surfels[mp.victim_idx];
        if (removed_ids.count(victim.id)) continue;
        if (removed_ids.count(mp.survivor->id)) continue;

        mp.survivor->W += victim.W;
        mp.survivor->S1 += victim.S1;
        mp.survivor->S2 += victim.S2;
        mp.survivor->reconstruct();

        mp.survivor->obs_count = std::max(mp.survivor->obs_count, victim.obs_count);
        mp.survivor->last_seen = std::max(mp.survivor->last_seen, victim.last_seen);

        // Delta update tracking
        removed_ids.insert(victim.id);
        deleted_ids_.insert(victim.id);
        updated_ids_.erase(victim.id);
        updated_ids_.insert(mp.survivor->id); // survivor was modified
        victim_voxel->remove_at(mp.victim_idx);
    }

    if (!pairs.empty()) cache_dirty_ = true;
}

FrameSurfel SurfelMap::transform_surfel_to_world(const FrameSurfel& fs, const Eigen::Isometry3f& pose) const {
    const Eigen::Matrix3f& R = pose.rotation();
    
    FrameSurfel fs_w;
    fs_w.sid = fs.sid;
    fs_w.centroid = pose * fs.centroid;
    fs_w.normal = R * fs.normal;
    fs_w.R = R * fs.R * R.transpose();
    fs_w.eigenvalues = fs.eigenvalues;
    fs_w.eigenvectors = R * fs.eigenvectors;
    fs_w.C_shape = R * fs.C_shape * R.transpose();
    fs_w.weight = fs.weight;
    fs_w.view_cos_theta = fs.view_cos_theta;

    return fs_w;
}

std::vector<MapSurfel*> SurfelMap::get_updated_surfels() {
    const auto& all = get_all_surfels();
    std::vector<MapSurfel*> result;
    result.reserve(updated_ids_.size());

    for (MapSurfel* ms : all) {
        if (updated_ids_.count(ms->id)) {
            result.push_back(ms);
        }
    }

    clear_deltas(); // Clear deltas after each call of get_updated_surfels()
    return result;
}

const std::vector<MapSurfel*>& SurfelMap::get_all_surfels() {
    if (!cache_dirty_) return surfel_cache_;

    surfel_cache_.clear();
    surfel_cache_.reserve(grid_->total_surfel_count());

    for (auto& [key, voxel] : *grid_) {
        for (auto& ms : voxel) {
            surfel_cache_.push_back(&ms);
        }
    }

    cache_dirty_ = false;
    return surfel_cache_;
}

}
