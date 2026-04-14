#include "surfel_map/surfel_map.hpp"

namespace smip_uav {
    
SurfelMap::SurfelMap(const Config& cfg) : cfg_(cfg) {
    builder_ = std::make_unique<FrameBuilder>(cfg_.builder_config);
    processor_ = std::make_unique<FrameProcessor>(cfg_.processor_config);
    grid_ = std::make_unique<VoxelGrid>(cfg_.grid_config);

    log_2pi_1_5_ = 1.5f * std::log(2.0f * static_cast<float>(M_PI));
    const float vs = cfg_.grid_config.voxel_size;
    const float V_voxel = vs * vs * vs;
    log_lambda_new_ = std::log(cfg_.pi_birth) - std::log(V_voxel);

    // lambda_max_ = std::pow(cfg_.grid_config.voxel_size * 0.3f, 2); // max eigenvalue (spatial extend) 0.3 * voxel size
    lambda_max_ = -1.0f;
}

void SurfelMap::update(const std::vector<PointXYZ>& scan, const Eigen::Isometry3f& pose, int64_t timestamp_ns, std::vector<FrameSurfel>* frame_surfels_out) {
    if (scan.empty()) return;

    GroundPlane gnd;
    gnd.normal_z = pose.rotation().transpose() * Eigen::Vector3f::UnitZ();
    gnd.offset_z = -gnd.normal_z.dot(pose.inverse().translation());
    frame_ = builder_->process(scan, timestamp_ns, &gnd);

    std::vector<FrameSurfel> surfels = processor_->process(frame_);
    if (frame_surfels_out) *frame_surfels_out = surfels;

    integrate(surfels, pose, timestamp_ns);

    frame_count_++;
    if (cfg_.merge_interval > 0 && (frame_count_ % cfg_.merge_interval) == 0) {
        merge();
    }
}

void SurfelMap::integrate(const std::vector<FrameSurfel>& frame_surfels, const Eigen::Isometry3f& pose, int64_t timestamp_ns) {
    std::unordered_map<MapSurfel*, ComponentAccum> accums;
    std::vector<FrameSurfel> birth_candidates;
    std::vector<RespEntry> resp;

    // E-Step: Compute responsibilities and accumulate
    for (const FrameSurfel& fs : frame_surfels) {
        FrameSurfel fs_w = transform_surfel_to_world(fs, pose);

        resp.clear();
        const float r_new = compute_responsibilities(fs_w, resp);
        const float w_k = fs_w.weight;

        // accumulate weighted observations into each responsible component
        for (const auto& entry : resp) {
            if (entry.r < 1e-6f) continue;

            auto& acc = accums[entry.component];
            const float wr = w_k * entry.r;
            acc.delta_W += wr;
            acc.delta_S1 += wr * fs_w.centroid;
            acc.delta_S2 += wr * (fs_w.centroid * fs_w.centroid.transpose() + fs_w.C_shape);
            acc.available_weight += w_k; // total evidence this component was tested against
        }

        if (r_new > cfg_.birth_residual) {
            birth_candidates.push_back(fs_w);
        }
    }

    // M-Step: Apply accumulated deltas, reconstruct params
    for (auto& [ms_ptr, acc] : accums) {
        const float gamma = 0.99f;
        ms_ptr->W = gamma * ms_ptr->W + acc.delta_W;
        ms_ptr->S1 = gamma * ms_ptr->S1 + acc.delta_S1;
        ms_ptr->S2 = gamma * ms_ptr->S2 + acc.delta_S2;
        // ms_ptr->W += acc.delta_W;
        // ms_ptr->S1 += acc.delta_S1;
        // ms_ptr->S2 += acc.delta_S2;
        ms_ptr->reconstruct(lambda_max_);
        ms_ptr->obs_count++;
        ms_ptr->last_seen = timestamp_ns;
    }

    // Health update: capture ratio for each tested component
    for (auto& [ms_ptr, acc] : accums) {
        if (acc.available_weight < 1e-8f) continue;
        
        const float c = acc.delta_W / acc.available_weight;
        ms_ptr->health = (1.0f - cfg_.health_alpha) * ms_ptr->health + cfg_.health_alpha * c; // EMA
        ms_ptr->n_eval++;
    }

    // Birth / Spawn new
    for (const FrameSurfel& fs_w : birth_candidates) {
        birth(fs_w, timestamp_ns);
    }

    // Prune: Remove components that lost competition
    prune();

    cache_dirty_ = true;
}

float SurfelMap::compute_responsibilities(const FrameSurfel& fs_w, std::vector<RespEntry>& resp_out) {
    const VoxelKey key = grid_->to_key(fs_w.centroid);

    // collect candidates from center voxel + 6-conn-nbs (modifies resp_out)
    auto search_voxel = [&](Voxel& voxel) {
        for (uint8_t i = 0; i < voxel.count; ++i) {
            MapSurfel& ms = voxel.surfels[i];

            // hard normal gate
            if (std::abs(fs_w.normal.dot(ms.normal)) < cfg_.normal_gate_cos) continue;

            // In compute_responsibilities, after extracting ms
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(ms.sigma);
            Eigen::Vector3f ev = eig.eigenvalues().cwiseMax(1e-6f);
            const Eigen::Matrix3f V = eig.eigenvectors();

            // Compress tangential directions for gating
            const float gate_tangential_cap = cfg_.grid_config.voxel_size * 0.15f; // tune this
            ev(1) = std::min(ev(1), gate_tangential_cap * gate_tangential_cap);
            ev(2) = std::min(ev(2), gate_tangential_cap * gate_tangential_cap);
            Eigen::Matrix3f sigma_gate = V * ev.asDiagonal() * V.transpose();

            // const Eigen::Matrix3f M = ms.sigma + fs_w.R;
            const Eigen::Matrix3f M = sigma_gate + fs_w.R;
            const Eigen::LDLT<Eigen::Matrix3f> M_ldlt(M);

            const Eigen::Vector3f nu = fs_w.centroid - ms.mu;
            const float epsilon = nu.dot(M_ldlt.solve(nu)); // nu * M^-1 * nu.T

            // log|M| from LDLT diag
            const float log_det_M = M_ldlt.vectorD().array().abs().log().sum();

            // log(r~)
            const float log_r_tilde = std::log(ms.W + 1e-10f) - log_2pi_1_5_ - 0.5 * log_det_M - 0.5f * epsilon;

            resp_out.push_back({&ms, log_r_tilde, 0.0f});
        }
    };

    if (Voxel* v = grid_->get(key)) search_voxel(*v); // search center voxel
    grid_->for_each_nb6(key, [&](const VoxelKey&, Voxel& v) { search_voxel(v); }); // search nb-6 voxels
    
    if (resp_out.empty()) {
        return 1.0f; // no candidates at all - entire resp goes to birth
    }

    // find max log value for numerical stability
    float max_log = log_lambda_new_;
    for (const auto& e : resp_out) {
        max_log = std::max(max_log, e.log_r_tilde);
    }

    // exponentiate and sum
    float sum = std::exp(log_lambda_new_ - max_log); // birht hypothesis
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

void SurfelMap::birth(const FrameSurfel& fs_w, int64_t timestamp_ns) {
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
    ms.health = 1.0f;
    ms.n_eval = 0;

    const VoxelKey key = grid_->to_key(ms.mu);
    Voxel& voxel = grid_->get_or_create(key);
    voxel.try_add(ms);
}

void SurfelMap::prune() {
    for (auto& [key, voxel] : *grid_) {
        for (uint8_t i = 0; i < voxel.count; ) {
            const MapSurfel& ms = voxel.surfels[i];
            if (ms.n_eval >= cfg_.eval_min && ms.health < cfg_.health_min) {
                voxel.remove_at(i); // swap-and-pop: Dont increment i here
            }
            else {
                ++i;
            }
        }
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
            
                if (std::abs(a.normal.dot(b.normal)) < cfg_.merge_normal_cos) continue; // hard normal gate

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

                    if (std::abs(a.normal.dot(b.normal)) < cfg_.merge_normal_cos) continue; // hard normal gate

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
        mp.survivor->reconstruct(lambda_max_);

        mp.survivor->obs_count = std::max(mp.survivor->obs_count, victim.obs_count);
        mp.survivor->last_seen = std::max(mp.survivor->last_seen, victim.last_seen);
        mp.survivor->health = std::max(mp.survivor->health, victim.health);
        mp.survivor->n_eval = std::max(mp.survivor->n_eval, victim.n_eval);

        removed_ids.insert(victim.id);
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
