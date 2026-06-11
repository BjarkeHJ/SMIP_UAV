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
    // Update rollover metrics
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
    const Eigen::Isometry3f T_local_sensor = T_origin_world_inv_ * frame.meta.pose.T_world;

    // Fuse frame surfels into active map surfels
    fuse_surfels(frame, T_local_sensor);
}

void ActiveMap::fuse_surfels(const Frame& frame, const Eigen::Isometry3f& T_local_sensor) {
    for (const Surfel& s_sensor : frame.surfels) {

        const Surfel s_local = transform_to_local(s_sensor, T_local_sensor);
        const VoxelKey home = to_coarse_key(s_local.position);

        // Collect ActiveSurfels from 27-cell nbhood
        candidate_scratch_.clear();
        collect_candidates(home, candidate_scratch_);

        ActiveSurfel* best = nullptr;
        float best_mahal = cfg_.max_mahalanobis_sq;

        for (ActiveSurfel* as : candidate_scratch_) {
            if (as->estimate.normal.dot(s_local.normal) < cfg_.min_normal_dot) continue;

            const Eigen::Vector3f dp = s_local.position - as->estimate.position;
            // Fast pre-filter: point-to-plane distance along the fused normal
            if (std::abs(as->estimate.normal.dot(dp)) >= cfg_.max_point_to_plane_m) continue;

            const Eigen::Matrix3f C_gate = as->estimate.covariance + s_local.covariance
                                         + cfg_.omega_regularization * Eigen::Matrix3f::Identity();
            const Eigen::LLT<Eigen::Matrix3f> llt(C_gate);
            if (llt.info() != Eigen::Success) continue;

            const float mahal_sq = dp.dot(llt.solve(dp));
            if (mahal_sq < best_mahal) {
                best_mahal = mahal_sq;
                best = as;
            }
        }
        
        if (best) {
            // Found suitable match
            fuse_into(*best, s_local);
            best->meta.unobserved_frames = 0;
            if (best->fusion.fuse_count >= cfg_.maturity_fuse_count) {
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
    const Eigen::LLT<Eigen::Matrix3f> llt_R(R_reg);
    const Eigen::Matrix3f R_inv = llt_R.solve(Eigen::Matrix3f::Identity());

    fs.Omega += R_inv;
    fs.xi += R_inv * s_local.position;
    fs.normal_acc += s_local.confidence * s_local.normal;
    fs.normal_weight += s_local.confidence;

    // Parallel-axis merge: combines the incoming shape with the accumulated shape,
    // adding the between-mean offset term so extent grows as viewpoints diverge.
    const float w_a = fs.shape_weight;
    const float w_b = s_local.confidence;
    const float w_tot = w_a + w_b;
    const Eigen::Vector3f d_mu = fs.shape_mu - s_local.position;
    fs.shape_acc = (w_a * fs.shape_acc + w_b * s_local.shape
                  + (w_a * w_b / w_tot) * d_mu * d_mu.transpose()) / w_tot;
    fs.shape_mu  = (w_a * fs.shape_mu + w_b * s_local.position) / w_tot;
    fs.shape_weight = w_tot;

    recompute_estimate(as);
    as.fusion.fuse_count++;
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
    as.fusion.shape_acc    = s_local.shape;
    as.fusion.shape_mu     = s_local.position;
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
        }
    }

}

void ActiveMap::finalize_estimate(ActiveSurfel& as) {
    Surfel& s = as.estimate;

    if (as.fusion.shape_weight > 1e-8f) {
        s.shape = as.fusion.shape_acc;
    }

    s.confidence = 1.0f - std::exp(-as.fusion.shape_weight / cfg_.confidence_weight_ref);
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

FrozenSubmap ActiveMap::freeze(int64_t stamp_ns_end) {
    evict_immature();

    FrozenSubmap fs;
    fs.T_submap_world = T_origin_world_;
    fs.T_submap_world_origin = T_origin_world_;
    fs.stamp_ns_start = stamp_ns_start_;
    fs.stamp_ns_end = stamp_ns_end;
    fs.frame_count = frame_count_;
    fs.accumulated_translation = accumulated_translation_;

    fs.surfels.reserve(total_surfel_count_);
    for (auto& [key, voxel] : voxel_map_) {
        for (ActiveSurfel& as : voxel.surfels) {
            finalize_estimate(as);
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

