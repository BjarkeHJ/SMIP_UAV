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

    // Integrate into SurfelMap
    integrate(surfels, pose, timestamp_ns);

    // Post-processing
}

void SurfelMap::integrate(const std::vector<FrameSurfel>& frame_surfels, const Eigen::Isometry3f& pose, int64_t timestamp_ns) {

    for (const FrameSurfel& fs : frame_surfels) {
        FrameSurfel fs_w = transform_surfel_to_world(fs, pose);
        MapSurfel* match = find_association(fs_w);

        if (match) {
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

MapSurfel* SurfelMap::find_association(const FrameSurfel& fs_world) {
    const VoxelKey key = grid_->to_key(fs_world.centroid);

    MapSurfel* best = nullptr;
    float best_d2 = cfg_.mahal_gate_sq;

    auto search_voxel = [&](Voxel& voxel) {
        for (uint8_t i = 0; i < voxel.count; ++i) {
            MapSurfel& ms = voxel.surfels[i];

            // Check normal vector alignment
            const float n_dot = std::abs(fs_world.normal.dot(ms.normal()));
            if (n_dot < cfg_.normal_gate_cos) continue;

            // Mahalanobis distance gating with geometric covariance
            const Eigen::Vector3f d = fs_world.centroid - ms.mu;
            const Eigen::Matrix3f S = ms.C_shape + fs_world.R;
            const float d2 = d.transpose() * S.inverse() * d;

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

void SurfelMap::fuse(MapSurfel& m_surfel, const FrameSurfel& f_surfel, int64_t timestamp_ns) {
    const Eigen::Vector3f& n = f_surfel.normal;

    // point-to-plane kalman update
    const float sigma2 = n.transpose() * f_surfel.R * n;
    const float r = n.dot(f_surfel.centroid - m_surfel.mu);
    const float S = n.transpose() * m_surfel.P * n + sigma2;
    const Eigen::Vector3f K = (m_surfel.P * n) / S;

    m_surfel.mu += K * r;
    m_surfel.P -= K * n.transpose() * m_surfel.P;

    // tangential correction
    const Eigen::Matrix3f T = Eigen::Matrix3f::Identity() - n * n.transpose();
    const Eigen::Vector3f d_tan = T * (f_surfel.centroid - m_surfel.mu);

    const Eigen::Matrix3f R_tan = cfg_.kappa * sigma2 * T + cfg_.epsilon * n * n.transpose();
    const Eigen::Matrix3f S_tan = m_surfel.P + R_tan;
    const Eigen::Matrix3f K_tan = m_surfel.P * S_tan.inverse();

    m_surfel.mu += K_tan * d_tan;
    m_surfel.P = (Eigen::Matrix3f::Identity() - K_tan) * m_surfel.P;

    // normal update
    m_surfel.normal_acc += f_surfel.normal * f_surfel.weight;
    m_surfel.normal_w += f_surfel.weight;

    m_surfel.obs_count++;
    m_surfel.last_seen = timestamp_ns;

    // shape
    const Eigen::Matrix3f C_fs = f_surfel.eigenvectors * f_surfel.eigenvalues.asDiagonal() * f_surfel.eigenvectors.transpose();
    const float w = f_surfel.weight;
    const float W_new = m_surfel.W_shape + w;
    m_surfel.C_shape = (m_surfel.W_shape * m_surfel.C_shape + w * C_fs) / W_new;
    m_surfel.W_shape = W_new;
}


FrameSurfel SurfelMap::transform_surfel_to_world(const FrameSurfel& fs, const Eigen::Isometry3f& pose) const {
    const Eigen::Matrix3f& Rot = pose.rotation();

    FrameSurfel fw;
    fw.sid = fs.sid;
    fw.centroid = pose * fs.centroid;
    fw.normal = Rot * fs.normal;
    fw.R = Rot * fs.R * Rot.transpose();
    fw.eigenvalues = fs.eigenvalues;
    fw.eigenvectors = Rot * fs.eigenvectors;
    fw.weight = fs.weight;
    fw.view_cos_theta = fs.view_cos_theta;

    return fw;
}

MapSurfel SurfelMap::create_map_surfel(const FrameSurfel& fs_world, int64_t timestamp_ns) {
    MapSurfel ms;
    ms.id = next_id_++;
    ms.mu = fs_world.centroid;
    ms.P = fs_world.R;
    ms.C_shape = fs_world.eigenvectors * fs_world.eigenvalues.asDiagonal() * fs_world.eigenvectors.transpose();
    ms.W_shape = fs_world.weight;
    ms.normal_acc = fs_world.normal * fs_world.weight;
    ms.normal_w = fs_world.weight;
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