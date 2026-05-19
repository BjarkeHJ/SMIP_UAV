#include "surfel_map/surfel_map.hpp"

namespace smip_uav {
    
SurfelMap::SurfelMap(const Config& cfg) : cfg_(cfg) {
    grid_ = std::make_unique<VoxelGrid>(cfg_.grid_config);

    // Constants computed by construction
    log_2pi_1_5_ = 1.5f * std::log(2.0f * static_cast<float>(M_PI));
    log_lambda_new_ = std::log(cfg_.spawn_intensity);
    inv_2_sigma_n_sq_ = 1.0f / (2.0f * cfg_.normal_sigma * cfg_.normal_sigma);
    merge_normal_cos_ = std::cos(cfg_.merge_normal_k * cfg_.normal_sigma );
}

void SurfelMap::update_map(const std::vector<FrameSurfel>& frame_surfels, const Eigen::Isometry3f& pose, int64_t timestamp_ns) {
    if (frame_surfels.empty()) return;
    
    frame_count_++;

    integrate(frame_surfels, pose, timestamp_ns);

    if (cfg_.merge_interval > 0 && (frame_count_ % cfg_.merge_interval) == 0) {
        merge();
    }

    revoxel_drifted_surfels();
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
            // if (entry.r < 0.25f) continue; // only merge into significant responsibility
            if (entry.r < 0.05f) continue; // only merge into significant responsibility

            // Update accumulated stats
            auto& acc = accums_[entry.component];
            const float wr = w_k * entry.r; // scale weight by repsonsibility
            acc.delta_W += wr;
            acc.delta_S1 += wr * fs_w.centroid;
            acc.delta_S2 += wr * (fs_w.centroid * fs_w.centroid.transpose() + fs_w.C_shape);
        }

        if (r_new > cfg_.spawn_residual) {
            spawn_candidates_.push_back(fs_w);
        }
    }

    // M-Step: Apply accumulated deltas, reconstruct params
    for (auto& [ms_ptr, acc] : accums_) {
        float alpha = std::max(0.001f, std::exp(-static_cast<float>(ms_ptr->obs_count) / static_cast<float>(cfg_.converge_obs_min)));
        float gamma = 1.0f - alpha;

        // evolve the map surfel
        ms_ptr->W  = gamma * ms_ptr->W  + alpha * acc.delta_W;
        ms_ptr->S1 = gamma * ms_ptr->S1 + alpha * acc.delta_S1;
        ms_ptr->S2 = gamma * ms_ptr->S2 + alpha * acc.delta_S2;

        // Persistent disk prior: inject a pseudo-observation whose covariance matches the
        // surfel's current tangential shape but replaces the normal eigenvalue with a tight
        // prior. Squishes the normal direction without limiting lateral extent.
        {
            constexpr float kDiskW  = 0.05f;  // pseudo-obs weight per frame
            constexpr float kSigmaN = 1e-8f;  // target normal variance ~(0.1mm)^2
            const Eigen::Vector3f& n  = ms_ptr->normal;
            const Eigen::Vector3f  mu = ms_ptr->S1 / ms_ptr->W;
            const Eigen::Matrix3f  disk = ms_ptr->sigma + (kSigmaN - ms_ptr->eigenvalues[0]) * (n * n.transpose());
            ms_ptr->W  += kDiskW;
            ms_ptr->S1 += kDiskW * mu;
            ms_ptr->S2 += kDiskW * (mu * mu.transpose() + disk);
        }

        // reconstruct the surfel from statistics
        ms_ptr->reconstruct();
        ms_ptr->obs_count++;
        ms_ptr->last_seen = timestamp_ns;

        if (!ms_ptr->converged && ms_ptr->obs_count >= cfg_.converge_obs_min) {
            if (ms_ptr->planarity() >= cfg_.converge_planarity) {
                ms_ptr->converged = true;
            }
        }

        updated_ids_.insert(ms_ptr->id);
    }

    // Spawn new
    for (const FrameSurfel& fs_w : spawn_candidates_) {
        spawn(fs_w, timestamp_ns);
    }

    // Update rolling local-map window with this frame's unique active voxel keys
    {
        std::unordered_set<VoxelKey, VoxelKeyHash> frame_key_set;
        for (const auto& [ms_ptr, _] : accums_)
            if (auto it = surfel_home_.find(ms_ptr->id); it != surfel_home_.end())
                frame_key_set.insert(it->second);
        for (const auto& fs_w : spawn_candidates_)
            frame_key_set.insert(grid_->to_key(fs_w.centroid));

        for (const VoxelKey& k : frame_key_set)
            local_map_voxels_[k]++;

        active_voxel_window_.push_back(
            std::vector<VoxelKey>(frame_key_set.begin(), frame_key_set.end()));

        if (static_cast<int32_t>(active_voxel_window_.size()) > cfg_.local_map_window) {
            for (const VoxelKey& k : active_voxel_window_.front()) {
                auto it = local_map_voxels_.find(k);
                if (it != local_map_voxels_.end() && --it->second == 0)
                    local_map_voxels_.erase(it);
            }
            active_voxel_window_.pop_front();
        }
    }

    std::cout << "Size of local map (n voxels): " << local_map_voxels_.size() << std::endl;

    cache_dirty_ = true;
}

float SurfelMap::compute_responsibilities(const FrameSurfel& fs_w, std::vector<RespEntry>& resp_out) {
    auto search_voxel = [&](Voxel& voxel) {
        for (uint8_t i = 0; i < voxel.count; ++i) {
            MapSurfel& ms = voxel.surfels[i];

            // Soft normal penalty: log N(theta; 0, sigma_n) via small-angle approx theta^2 ~ 2(1-cos)
            const float dot_n = fs_w.normal.dot(ms.normal);
            const float log_p_normal = -2.0f * (1.0f - dot_n) * inv_2_sigma_n_sq_;

            // M: marginal covariance of the observation given the component
            const Eigen::Matrix3f M = ms.sigma + fs_w.R;
            const Eigen::LDLT<Eigen::Matrix3f> M_ldlt(M);

            const Eigen::Vector3f nu = fs_w.centroid - ms.mu;
            const float epsilon = nu.dot(M_ldlt.solve(nu)); // nu * M^-1 * nu.T

            // log|M| from LDLT diag
            const float log_det_M = M_ldlt.vectorD().array().abs().log().sum();

            // log(r~)
            const float log_r_tilde = std::log(ms.W + 1e-7f) - log_2pi_1_5_ - 0.5f*log_det_M - 0.5f*epsilon + log_p_normal;

            // {ptr to ms, log responsibility, 0.0f (normalize later)}
            resp_out.push_back({&ms, log_r_tilde, 0.0f});
        }
    };

    // Lookup the Surfel center in the VoxelGrid and search 
    const VoxelKey key = grid_->to_key(fs_w.centroid);
    if (Voxel* v = grid_->get(key)) search_voxel(*v);
    grid_->for_each_nb6(key, [&](const VoxelKey&, Voxel& v) { search_voxel(v); });

    if (resp_out.empty()) {
        return 1.0f; // no candidates at all - entire resp goes to spawn
    }

    // Normalize component log-priors by local neighbourhood total weight so that
    // log(W_j) -> log(W_j / sum_W), keeping the fixed spawn hypothesis calibrated
    float sum_W = 0.0f;
    for (const auto& e : resp_out) {
        sum_W += e.component->W;
    }
    const float log_sum_W = std::log(sum_W + 1e-10f);
    for (auto& e : resp_out) {
        e.log_r_tilde -= log_sum_W;
    }

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
    ms.W = w + cfg_.prior_W;
    // ms.S1 = w * fs_w.centroid;
    ms.S1 = w * fs_w.centroid + cfg_.prior_W * fs_w.centroid;
    // ms.S2 = w * (fs_w.centroid * fs_w.centroid.transpose() + fs_w.C_shape);
    ms.S2 = w * (fs_w.centroid * fs_w.centroid.transpose() + fs_w.C_shape) + cfg_.prior_W * (fs_w.centroid * fs_w.centroid.transpose() + cfg_.prior_S2_scale);
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
        surfel_home_.erase(evicted_id);
    }

    if (voxel.try_add(ms)) {
        surfel_home_[ms.id] = key;
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

    const float tau_n = 0.1f * cfg_.grid_config.voxel_size;
    auto should_merge = [&](const MapSurfel& a, const MapSurfel& b) -> bool {
        // if (a.converged || b.converged) return false;

        // normal alignment
        if (a.normal.dot(b.normal) < merge_normal_cos_) return false;
        
        const Eigen::Vector3f d = a.mu - b.mu;

        // Coplanarity gate (normal direction only)
        const Eigen::Vector3f n_avg = (a.normal + b.normal).normalized();
        if (std::abs(d.dot(n_avg)) > tau_n) return false;

        // In-plane Mahalanobis distance — project out normal so the near-zero
        // normal eigenvalue of sigma doesn't dominate and make this gate useless.
        const Eigen::Vector3f d_tan = d - d.dot(n_avg) * n_avg;
        const Eigen::Matrix3f S = a.sigma + b.sigma;
        const float d2 = d_tan.dot(S.ldlt().solve(d_tan));
        if (d2 >= cfg_.merge_mahal_sq) return false;

        
        // Predict merege planarity
        if (a.planarity() < cfg_.merge_min_planarity * 0.8f || b.planarity() < cfg_.merge_min_planarity * 0.8f) return false;

        const float W_new = a.W + b.W;
        const Eigen::Vector3f mu_new = (a.W * a.mu + b.W * b.mu) / W_new;
        const Eigen::Vector3f da = a.mu - mu_new;
        const Eigen::Vector3f db = b.mu - mu_new;
        const Eigen::Matrix3f sigma_new = (a.W * (a.sigma + da * da.transpose()) + b.W * (b.sigma + db * db.transpose())) / W_new;
        
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(sigma_new);
        if (eig.info() != Eigen::Success) return false;
        const auto& ev = eig.eigenvalues();
        const float planarity_pred = (ev(1) - ev(0)) / (ev(2) + 1e-10f);
        if (planarity_pred < cfg_.merge_min_planarity) return false;

        return true;
    };

    const float planarity_floor = cfg_.merge_min_planarity * 0.8f;

    for (const auto& [key_a, _] : local_map_voxels_) {
        Voxel* vp_a = grid_->get(key_a);
        if (!vp_a) continue;
        Voxel& voxel_a = *vp_a;

        // Delete stable but non-planar surfels (enough observations yet still below floor = noise)
        for (int8_t i = (int8_t)voxel_a.count - 1; i >= 0; --i) {
            const MapSurfel& ms = voxel_a.surfels[i];
            if (ms.obs_count >= cfg_.converge_obs_min && ms.planarity() < planarity_floor) {
                deleted_ids_.insert(ms.id);
                updated_ids_.erase(ms.id);
                surfel_home_.erase(ms.id);
                voxel_a.remove_at((uint8_t)i);
                cache_dirty_ = true;
            }
        }
        if (voxel_a.empty()) continue;

        // Intra-voxel pairs
        for (uint8_t i = 0; i < voxel_a.count; ++i) {
            for (uint8_t j = i + 1; j < voxel_a.count; ++j) {
                MapSurfel& a = voxel_a.surfels[i];
                MapSurfel& b = voxel_a.surfels[j];
                if (!should_merge(a,b)) continue;

                if (a.W >= b.W) {
                    pairs.push_back({&a, key_a, j});
                }
                else {
                    pairs.push_back({&b, key_a, i});
                }
            }
        }

        // Cross-voxel pairs (center vs each nb6)
        // grid_->for_each_nb26(key_a, [&](const VoxelKey& key_b, Voxel& voxel_b) {
        grid_->for_each_nb6(key_a, [&](const VoxelKey& key_b, Voxel& voxel_b) {
            // only process if key_a < key_b to avoid doubles
            if (!(key_a < key_b)) return;

            for (uint8_t i = 0; i < voxel_a.count; ++i) {
                for (uint8_t j = 0; j < voxel_b.count; ++j) {
                    MapSurfel& a = voxel_a.surfels[i];
                    MapSurfel& b = voxel_b.surfels[j];
                    if (!should_merge(a,b)) continue;

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
        surfel_home_.erase(victim.id);
        updated_ids_.insert(mp.survivor->id); // survivor was modified
        victim_voxel->remove_at(mp.victim_idx);
    }

    if (!pairs.empty()) cache_dirty_ = true;
}

void SurfelMap::revoxel_drifted_surfels() {
    struct MoveTask {
        MapSurfel surfel;  // full copy taken before any grid mutation
        VoxelKey  old_key;
    };
    std::vector<MoveTask> tasks;

    for (const auto& [ms_ptr, _] : accums_) {
        const auto it = surfel_home_.find(ms_ptr->id);
        if (it == surfel_home_.end()) continue;

        const VoxelKey old_key = it->second;
        const VoxelKey new_key = grid_->to_key(ms_ptr->mu);
        if (new_key == old_key) continue;

        tasks.push_back({ *ms_ptr, old_key });
    }

    for (auto& t : tasks) {
        const VoxelKey new_key = grid_->to_key(t.surfel.mu);

        // Check capacity in target voxel before committing to the move.
        // If the mover can't win eviction, leave it in its current (slightly wrong) voxel.
        Voxel* new_voxel_ptr = grid_->get(new_key);
        if (new_voxel_ptr && new_voxel_ptr->full()) {
            uint8_t min_idx = 0;
            for (uint8_t i = 1; i < new_voxel_ptr->count; ++i) {
                if (new_voxel_ptr->surfels[i].W < new_voxel_ptr->surfels[min_idx].W) min_idx = i;
            }
            if (t.surfel.W <= new_voxel_ptr->surfels[min_idx].W) continue;

            deleted_ids_.insert(new_voxel_ptr->surfels[min_idx].id);
            updated_ids_.erase(new_voxel_ptr->surfels[min_idx].id);
            surfel_home_.erase(new_voxel_ptr->surfels[min_idx].id);
            new_voxel_ptr->remove_at(min_idx);
        }

        // Remove from old voxel — search by id since swap-with-last may have shifted indices
        if (Voxel* old_voxel = grid_->get(t.old_key)) {
            for (uint8_t i = 0; i < old_voxel->count; ++i) {
                if (old_voxel->surfels[i].id == t.surfel.id) {
                    old_voxel->remove_at(i);
                    break;
                }
            }
        }
        deleted_ids_.insert(t.surfel.id);
        updated_ids_.erase(t.surfel.id);
        surfel_home_.erase(t.surfel.id);

        // Insert into the correct voxel
        Voxel& new_voxel = grid_->get_or_create(new_key);
        if (MapSurfel* inserted = new_voxel.try_add(t.surfel)) {
            surfel_home_[inserted->id] = new_key;
            updated_ids_.insert(inserted->id);
        }
    }
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
    // const auto& all = get_all_surfels();
    std::vector<MapSurfel*> result;
    result.reserve(updated_ids_.size());

    for (uint32_t id : updated_ids_) {
        auto it = surfel_home_.find(id);
        if (it == surfel_home_.end()) continue;
        if (Voxel* v = grid_->get(it->second)) {
            for (auto& ms : *v) {
                if (ms.id == id) {
                    result.push_back(&ms);
                    break;
                }
            }
        }
    }

    // for (MapSurfel* ms : all) {
    //     if (updated_ids_.count(ms->id)) {
    //         result.push_back(ms);
    //     }
    // }

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
