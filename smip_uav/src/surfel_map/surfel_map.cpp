#include "surfel_map/surfel_map.hpp"

namespace smip_uav {

SurfelMap::SurfelMap(const Config& cfg) : cfg_(cfg) {
    builder_ = std::make_unique<FrameBuilder>(cfg_.builder_config);
    processor_ = std::make_unique<FrameProcessor>(cfg_.processor_config);
    grid_ = std::make_unique<VoxelGrid>(cfg_.grid_config);
}

void SurfelMap::update(const std::vector<PointXYZ>& scan, const Eigen::Isometry3f& pose, int64_t timestamp_ns, std::vector<FrameSurfel>* frame_surfels_out) {
    if (scan.empty()) return;
    // check pose also?

    // Build frame from scan
    GroundPlane gnd;
    gnd.normal_z = pose.rotation().transpose() * Eigen::Vector3f::UnitZ();
    gnd.offset_z = -gnd.normal_z.dot(pose.inverse().translation());
    frame_ = builder_->process(scan, timestamp_ns, &gnd);

    // Process frame to extract local surfels
    std::vector<FrameSurfel> surfels = processor_->process(frame_);
    if (frame_surfels_out) *frame_surfels_out = surfels;

    // Correct pose (Point-to-plane ICP) of surfels
    // TODO

    // Integrate into SurfelMap
    integrate(surfels, pose, timestamp_ns);

    // Post-processing
}

void SurfelMap::integrate(const std::vector<FrameSurfel>& frame_surfels, const Eigen::Isometry3f& pose, int64_t timestamp_ns) {

    for (const FrameSurfel& fs : frame_surfels) {
        FrameSurfel fs_w = transform_surfel_to_world(fs, pose);
        MapSurfel* match = find_association(fs_w);

        if (match) {
            const double dt_s = (timestamp_ns - match->last_seen) * 1e-9;
            if (dt_s > 0.0) {
                match->P.diagonal().array() += cfg_.process_noise_rate * dt_s;
            }
            
            fuse(*match, fs_w, timestamp_ns);
        }
        else {
            MapSurfel ms = create_map_surfel(fs_w, timestamp_ns);
            const VoxelKey key = grid_->to_key(ms.mu);
            Voxel& voxel = grid_->get_or_create(key);
            voxel.try_add(ms);
        }
    }

    cache_dirty_ = true;
}

MapSurfel* SurfelMap::find_association(const FrameSurfel& fs_w) {
    const VoxelKey key = grid_->to_key(fs_w.centroid);

    MapSurfel* best = nullptr;
    float best_d2 = cfg_.mahal_gate_sq;

    auto search_voxel = [&](Voxel& voxel) {
        for (uint8_t i = 0; i < voxel.count; ++i) {
            MapSurfel& ms = voxel.surfels[i];

            // Check normal vector alignment
            const float n_dot = std::abs(fs_w.normal.dot(ms.normal));
            if (n_dot < cfg_.normal_gate_cos) continue;

            // Mahalanobis distance gating with geometric covariance
            const Eigen::Vector3f d = fs_w.centroid - ms.mu;
            const Eigen::Matrix3f S = ms.C_shape + ms.P + fs_w.R; // C_shape + P + R
            const float d2 = d.dot(S.ldlt().solve(d)); // faster than dS^(-1)d^T

            if (d2 < best_d2) {
                best_d2 = d2;
                best = &ms;
            }
        }
    };

    // Search center voxel
    if (Voxel* v = grid_->get(key)) {
        search_voxel(*v);
    }

    // Search 6-neighbors
    grid_->for_each_nb6(key, [&](const VoxelKey&, Voxel& v) {
        search_voxel(v);
    });

    return best;
}

void SurfelMap::fuse(MapSurfel& ms, const FrameSurfel& fs_w, int64_t timestamp_ns) {
    // Fuse a new (transformed) frame surfel into existing/associated map surfel
    const Eigen::Vector3f& n = fs_w.normal;
    ms.normal = n;

    // Anisotropic observation covariance
    const float sigma2_n = n.transpose() * fs_w.R * n;
    const Eigen::Matrix3f T = Eigen::Matrix3f::Identity() - n * n.transpose();
    const Eigen::Matrix3f R_obs = fs_w.R + cfg_.kappa * sigma2_n * T;

    // Kalman update
    const Eigen::Matrix3f S = ms.P + R_obs;
    const Eigen::LDLT<Eigen::Matrix3f> S_ldlt(S); // precomute S^-1
    const Eigen::Vector3f innovation = fs_w.centroid - ms.mu;
    const Eigen::Vector3f K_innov = ms.P * S_ldlt.solve(innovation);
    const Eigen::Matrix3f KP = ms.P * S_ldlt.solve(ms.P); // P S^-1 P

    ms.mu += K_innov;
    ms.P -= KP;

    // Shape update with parallel-axis correction
    const Eigen::Vector3f delta = fs_w.centroid - ms.mu; // after position update
    const float w = fs_w.weight;
    const float W_new = ms.W_shape + w;
    ms.C_shape = (ms.W_shape * ms.C_shape + w * (fs_w.C_shape + delta * delta.transpose())) / W_new;
    ms.W_shape = W_new;

    // Recompute cached normal from updated C_shape; use observation normal for sign
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(ms.C_shape);
    ms.normal = eig.eigenvectors().col(0);
    if (ms.normal.dot(n) < 0.0f) ms.normal = -ms.normal;

    ms.obs_count++;
    ms.last_seen = timestamp_ns;
}


FrameSurfel SurfelMap::transform_surfel_to_world(const FrameSurfel& fs, const Eigen::Isometry3f& pose) const {
    const Eigen::Matrix3f& Rot = pose.rotation();

    FrameSurfel fs_w;
    fs_w.sid = fs.sid;
    fs_w.centroid = pose * fs.centroid;
    fs_w.normal = Rot * fs.normal;
    fs_w.R = Rot * fs.R * Rot.transpose();
    fs_w.eigenvalues = fs.eigenvalues;
    fs_w.eigenvectors = Rot * fs.eigenvectors;
    fs_w.C_shape = Rot * fs.C_shape * Rot.transpose();
    fs_w.weight = fs.weight;
    fs_w.view_cos_theta = fs.view_cos_theta;

    return fs_w;
}

MapSurfel SurfelMap::create_map_surfel(const FrameSurfel& fs_w, int64_t timestamp_ns) {
    MapSurfel ms;
    ms.id = next_id_++;
    ms.mu = fs_w.centroid;
    ms.P = fs_w.R;
    ms.C_shape = fs_w.C_shape;
    ms.normal = fs_w.normal;
    ms.W_shape = fs_w.weight;
    ms.obs_count = 1;
    ms.last_seen = timestamp_ns;
    return ms;
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

} // smip_uav