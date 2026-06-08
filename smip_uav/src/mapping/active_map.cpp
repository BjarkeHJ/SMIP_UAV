#include "mapping/active_map.hpp"
#include <cassert>
#include <cmath>
#include <Eigen/Eigenvalues>

#include <iostream> // debug

namespace smip_uav {

ActiveMap::ActiveMap(const Eigen::Isometry3f& T_origin_world, int64_t stamp_ns)
    : ActiveMap(T_origin_world, stamp_ns, Config{}) {}

ActiveMap::ActiveMap(const Eigen::Isometry3f& T_origin_world, int64_t stamp_ns, const Config& cfg)
    : cfg_(cfg)
    , T_origin_world_(T_origin_world)
    , T_origin_world_inv_(T_origin_world.inverse())
    , T_latest_world_(T_origin_world)
    , stamp_ns_start_(stamp_ns)
{
    candidate_scratch_.reserve(64);
}

void ActiveMap::register_frame(const Frame& frame, const StampedPose& stamped_pose) {
    // Update ollover metrics
    if (frame_count_ > 0) {
        const Eigen::Vector3f dt = stamped_pose.T_world.translation() - T_latest_world_.translation();
        accumulated_translation_ += dt.norm();

        const Eigen::Matrix3f dR = T_latest_world_.rotation().transpose() * stamped_pose.T_world.rotation();
        const float cos_a = std::clamp((dR.trace() - 1.0f) * 0.5f, -1.0f, 1.0f);
        accumulated_rotation_rad_ += std::acos(cos_a);
    }

    T_latest_world_ = stamped_pose.T_world;
    ++frame_count_;

    // Tick unobserved counters
    tick_unobserved();

    // Compute transform: sensor frame -> submap-local frame
    const Eigen::Isometry3f T_local_sensor = T_origin_world_inv_ * frame.meta.T_sensor_world;

    // Insert valid pixels into subvoxel point grids
    insert_pixels(frame, T_local_sensor);

    // Fuse frame surfels into active map surfels
    fuse_surfels(frame, T_local_sensor);
}

void ActiveMap::insert_pixels(const Frame& frame, const Eigen::Isometry3f& T_local_sensor) {
    const Eigen::Matrix3f R_local = T_local_sensor.rotation(); 
    const size_t N = frame.pixels.pointnormals.size();

    for (size_t i = 0; i < N; ++i) {
        if (!frame.pixels.validities[i]) continue;
        const float w = frame.pixels.weights[i];
        if (w < 1e-6f) continue;

        const Eigen::Vector3f p_local = T_local_sensor * frame.pixels.pointnormals[i].p;
        const Eigen::Vector3f n_local = R_local * frame.pixels.pointnormals[i].n;

        const VoxelKey& ck = to_coarse_key(p_local);
        Voxel& voxel = voxel_map_[ck];

        // set voxel origin on first use
        if (!voxel.origin_set) {
            voxel.origin = Eigen::Vector3f(
                static_cast<float>(ck.x)*cfg_.voxel_size,
                static_cast<float>(ck.y)*cfg_.voxel_size,
                static_cast<float>(ck.z)*cfg_.voxel_size
            );
            voxel.origin_set = true;
        }

        // Compute subvoxel key relative to voxel origin
        const SubKey sk = to_sub_key(p_local, voxel.origin);
        
        // Running mean update
        PointBin& bin = voxel.point_bins[sk];
        if (bin.count == 0) ++total_bin_count_;
        const float w_total = bin.weight + w;
        bin.position = (bin.weight * bin.position + w * p_local) / w_total;
        bin.normal = (bin.weight * bin.normal + w * n_local) / w_total;
        bin.weight = w_total;
        bin.count++;
    }
}

void ActiveMap::fuse_surfels(const Frame& frame, const Eigen::Isometry3f& T_local_sensor) {
    for (const Surfel& s_sensor : frame.surfels) {
        const Surfel s_local = transform_to_local(s_sensor, T_local_sensor);
        const VoxelKey home = to_coarse_key(s_local.position);

        // Collect ActiveSurfels from 27-cell nbhood
        candidate_scratch_.clear();
        collect_candidates(home, candidate_scratch_);

        ActiveSurfel* best = nullptr;
        float best_d2p = cfg_.max_point_to_plane_m;

        for (ActiveSurfel* as : candidate_scratch_) {
            const float ndot = as->estimate.normal.dot(s_local.normal);
            if (ndot < cfg_.min_normal_dot) continue; // Not normal aligned

            const float d2p = std::abs(as->estimate.normal.dot(s_local.position - as->estimate.position));
            if (d2p >= cfg_.max_point_to_plane_m) continue; // not in-plane

            const Eigen::Matrix3f C_gate = as->estimate.covariance + s_local.covariance + cfg_.omega_regularization * Eigen::Matrix3f::Identity();
            // const Eigen::Matrix3f C_gate = as->estimate.shape + s_local.shape + cfg_.omega_regularization * Eigen::Matrix3f::Identity();
            const Eigen::Vector3f dp = s_local.position - as->estimate.position;
            const Eigen::LLT<Eigen::Matrix3f> llt(C_gate);
            if (llt.info() != Eigen::Success) continue;
            if (dp.dot(llt.solve(dp)) > cfg_.max_mahalanobis_sq) continue;
            
            if (d2p < best_d2p) {
                best_d2p = d2p;
                best = as;
            }
        }
        
        if (best) {
            // Found suitable match
            fuse_into(*best, s_local);
            best->meta.unobserved_frames = 0;
            if (best->estimate.obs_count >= cfg_.maturity_obs_count) {
                best->meta.mature = true;
            }
        }
        else {
            // New surface
            Voxel& voxel = voxel_map_[home];
            insert_surfel(home, voxel, s_local);
        }
    }
}

void ActiveMap::collect_candidates(const VoxelKey& home, std::vector<ActiveSurfel*>& out) {
    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const VoxelKey nb{home.x + dx, home.y + dy, home.z + dz};
                auto it = voxel_map_.find(nb);
                if (it == voxel_map_.end()) continue;
                for (ActiveSurfel& as : it->second.surfels) {
                    out.push_back(&as);
                }
            }
        }
    }
}

bool ActiveMap::fuse_into(ActiveSurfel& as, const Surfel& s_local) {
    FusionState& fs = as.fusion;

    const Eigen::Matrix3f R_reg = s_local.covariance + cfg_.omega_regularization * Eigen::Matrix3f::Identity();
    const Eigen::Matrix3f R_inv = R_reg.inverse();

    fs.Omega += R_inv;
    fs.xi += R_inv * s_local.position;
    fs.normal_acc += s_local.confidence * s_local.normal;
    fs.normal_weight += s_local.confidence;
    fs.shape_acc += s_local.confidence * s_local.shape;
    fs.shape_weight += s_local.confidence;

    recompute_estimate(as);
    as.estimate.obs_count++;
    return true;
}

void ActiveMap::insert_surfel(const VoxelKey&, Voxel& voxel, const Surfel& s_local) {
    ActiveSurfel as;
    as.estimate = s_local;
    const Eigen::Matrix3f R_reg = s_local.covariance + cfg_.omega_regularization * Eigen::Matrix3f::Identity();
    as.fusion.Omega = R_reg.inverse();
    as.fusion.xi = as.fusion.Omega * s_local.position;
    as.fusion.normal_acc = s_local.confidence * s_local.normal;
    as.fusion.normal_weight = s_local.confidence;
    as.fusion.shape_acc = s_local.confidence * s_local.shape;
    as.fusion.shape_weight = s_local.confidence;

    voxel.surfels.push_back(std::move(as));
    ++total_surfel_count_;
}

void ActiveMap::recompute_estimate(ActiveSurfel& as) {
    const FusionState& fs = as.fusion;
    Surfel& s = as.estimate;

    const Eigen::Matrix3f Omega_reg = fs.Omega + cfg_.omega_regularization * Eigen::Matrix3f::Identity();
    const Eigen::LLT<Eigen::Matrix3f> llt(Omega_reg);
    if (llt.info() != Eigen::Success) return;

    s.covariance = llt.solve(Eigen::Matrix3f::Identity());
    s.position = s.covariance * fs.xi;

    if (fs.normal_weight > 1e-8f) {
        const Eigen::Vector3f n = fs.normal_acc / fs.normal_weight;
        const float nn = n.norm();
        if (nn > 1e-6f) {
            s.normal = n / nn;
            if (s.normal.dot(s.position) > 0.0f) {
                s.normal = -s.normal;
            }
        }
    }

    if (fs.shape_weight > 1e-8f) {
        s.shape = fs.shape_acc / fs.shape_weight;
    }

    const float tr = s.covariance.trace();
    s.confidence = (1.0f - std::exp(-static_cast<float>(s.obs_count) * 0.5f)) * std::exp(-tr * 10.0f);
}

void ActiveMap::tick_unobserved() {
    for (auto& [key, voxel] : voxel_map_) {
        auto& sv = voxel.surfels;
        for (size_t i = 0; i < sv.size(); ) {
            ActiveSurfel& as = sv[i];
            if (as.meta.mature) {
                ++i;
                continue;
            }

            ++as.meta.unobserved_frames;
            if (as.meta.unobserved_frames > cfg_.max_unobserved_frames) {
                // swap-erase - order within voxel does not matter
                sv[i] = std::move(sv.back());
                sv.pop_back();
                --total_surfel_count_;
            }
            else {
                ++i;
            }
        }
    }
}

void ActiveMap::evict_immature() {
    for (auto& [key, voxel] : voxel_map_) {
        auto& sv = voxel.surfels;
        size_t i = 0;
        while (i < sv.size()) {
            if (!sv[i].meta.mature) {
                sv[i] = std::move(sv.back());
                sv.pop_back();
                --total_surfel_count_;
            }
            else {
                ++i;
            }
        }
    }
}

void ActiveMap::smooth_surfels() {
    // One pass of tangent-plane-constrained Laplacian smoothing.
    // For each surfel, compute weighted mean of spatial neighbours,
    // then move only the tangent-plane component of the displacement.
    //
    // Neighbour search: same 27-cell voxel neighbourhood used for correspondence.
    // Weight: inverse squared distance, confidence-weighted.
    // Constraint: displacement projected onto tangent plane only —
    //             normal direction is untouched.

    const float search_radius_sq =
        cfg_.voxel_size * cfg_.voxel_size;  // neighbours within one voxel cell

    // Collect smoothed positions first, apply after (Jacobi-style, not Gauss-Seidel)
    // so smoothing is order-independent.
    struct SmoothedResult {
        Eigen::Vector3f new_position;
        bool updated{false};
    };

    // Flat list of all active surfels with their voxel keys for fast iteration
    std::vector<std::pair<VoxelKey, ActiveSurfel*>> all_surfels;
    all_surfels.reserve(total_surfel_count_);
    for (auto& [key, voxel] : voxel_map_)
        for (auto& as : voxel.surfels)
            all_surfels.push_back({key, &as});

    std::vector<Eigen::Vector3f> new_positions(all_surfels.size());

    for (size_t i = 0; i < all_surfels.size(); ++i) {
        const auto& [home_key, as] = all_surfels[i];
        const Surfel& s = as->estimate;

        // Collect neighbours from 27-cell search
        candidate_scratch_.clear();
        collect_candidates(home_key, candidate_scratch_);

        Eigen::Vector3f weighted_pos = Eigen::Vector3f::Zero();
        float total_weight = 0.0f;

        for (const ActiveSurfel* nb : candidate_scratch_) {
            if (nb == as) continue;  // skip self

            const Eigen::Vector3f dp = nb->estimate.position - s.position;
            const float dist_sq = dp.squaredNorm();
            if (dist_sq > search_radius_sq || dist_sq < 1e-8f) continue;

            // Normal compatibility: only smooth with surfels on the same surface
            if (nb->estimate.normal.dot(s.normal) < cfg_.min_normal_dot) continue;

            const float w = nb->estimate.confidence / (dist_sq + 1e-6f);
            weighted_pos += w * nb->estimate.position;
            total_weight += w;
        }

        if (total_weight < 1e-8f) {
            new_positions[i] = s.position;  // no neighbours — keep as-is
            continue;
        }

        const Eigen::Vector3f mean_pos = weighted_pos / total_weight;
        const Eigen::Vector3f delta    = mean_pos - s.position;

        // Project delta onto tangent plane — remove normal component
        const Eigen::Vector3f delta_tangent =
            delta - delta.dot(s.normal) * s.normal;

        // Damping: don't move more than half the search radius
        const float max_move = 0.5f * cfg_.voxel_size;
        const float move_len = delta_tangent.norm();
        const Eigen::Vector3f clamped_delta = (move_len > max_move)
            ? delta_tangent * (max_move / move_len)
            : delta_tangent;

        new_positions[i] = s.position + clamped_delta;
    }

    // Apply smoothed positions
    for (size_t i = 0; i < all_surfels.size(); ++i)
        all_surfels[i].second->estimate.position = new_positions[i];
}

FrozenSubmap ActiveMap::freeze(int64_t stamp_ns_end) {
    evict_immature();
    smooth_surfels();
    
    FrozenSubmap fs;
    fs.T_submap_world = T_origin_world_;
    fs.T_submap_world_origin = T_origin_world_;
    fs.stamp_ns_start = stamp_ns_start_;
    fs.stamp_ns_end = stamp_ns_end;
    fs.frame_count = frame_count_;
    fs.accumulated_translation = accumulated_translation_;

    fs.surfels.reserve(total_surfel_count_);
    for (const auto& [key, voxel] : voxel_map_) {
        for (const ActiveSurfel& as : voxel.surfels) {
            fs.surfels.push_back(as.estimate);
        }
    }

    fs.compute_bounds();
    fs.build_kdtree();

    return fs;
}

// HELPERS
ActiveMap::VoxelKey ActiveMap::to_coarse_key(const Eigen::Vector3f& p) const {
    const float inv = 1.0f / cfg_.voxel_size;
    return {
        static_cast<int32_t>(std::floor(p.x() * inv)),
        static_cast<int32_t>(std::floor(p.y() * inv)),
        static_cast<int32_t>(std::floor(p.z() * inv))
    };
}

ActiveMap::SubKey ActiveMap::to_sub_key(const Eigen::Vector3f& p_local, const Eigen::Vector3f& voxel_origin) const {
    const Eigen::Vector3f rel = p_local - voxel_origin;
    const float inv = 1.0f / cfg_.subvoxel_size;
    return {
        static_cast<int16_t>(std::floor(rel.x() * inv)),
        static_cast<int16_t>(std::floor(rel.y() * inv)),
        static_cast<int16_t>(std::floor(rel.z() * inv))
    };
}

Surfel ActiveMap::transform_to_local(const Surfel& s, const Eigen::Isometry3f& T_local_sensor) const {
    Surfel out = s;
    out.position = T_local_sensor * s.position;
    const Eigen::Matrix3f R = T_local_sensor.rotation();
    out.normal = R * s.normal;
    out.covariance = R * s.covariance * R.transpose();
    out.shape = R * s.shape * R.transpose();
    return out;
}


} // namespace smip_uav

