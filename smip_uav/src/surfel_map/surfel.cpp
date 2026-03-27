#include "surfel_map/surfel.hpp"

namespace smip_uav {

void Surfel::seed(const PointNormal& pn, const Eigen::Vector3f& view_dir, const VoxelKey& key, int64_t timestamp) {
    const float w = std::max(pn.w, 1e-6f);

    const Eigen::Vector3f p = {pn.px, pn.py, pn.pz};
    const Eigen::Vector3f n = {pn.nx, pn.ny, pn.nz};

    W_ = w;
    S1_ = w * p;
    S2_ = w * p * p.transpose();
    count_ = 1;

    sum_view_dir_ = view_dir;

    key_ = key;

    mean_ = p;
    normal_ = n;
    if (normal_.dot(view_dir) < 0.0f) normal_ = -normal_;

    ts_create_ = timestamp;
    ts_update_ = timestamp;
    dirty_ = true;
}

void Surfel::accumulate(const PointNormal& pn, const Eigen::Vector3f& view_dir, int64_t timestamp) {
    const float w = std::max(pn.w, 1e-6f);
    const Eigen::Vector3f p = {pn.px, pn.py, pn.pz};

    W_ += w;
    S1_ += w * p;
    S2_ += w * p * p.transpose();
    count_++;

    sum_view_dir_ += view_dir;
    ts_update_ = timestamp;
    dirty_ = true;
}

void Surfel::merge_from(const Surfel& other) {
    W_ += other.W_;
    S1_ += other.S1_;
    S2_ += other.S2_;
    count_ += other.count_;

    sum_view_dir_ += other.sum_view_dir_;

    ts_create_ = std::min(ts_create_, other.ts_create_);
    ts_update_ = std::max(ts_update_, other.ts_update_);
    dirty_ = true;
}

void Surfel::merge_from_translated(const Surfel& other, const Eigen::Vector3f& delta_c) {
    const Eigen::Vector3f S1_t = other.S1_ + other.W_ * delta_c;
    const Eigen::Matrix3f S2_t = other.S2_ 
                                + other.S1_ * delta_c.transpose()
                                + delta_c * other.S1_.transpose()
                                + other.W_ * delta_c * delta_c.transpose();

    W_ += other.W_;
    S1_ += S1_t;
    S2_ += S2_t;
    count_ += other.count_;

    sum_view_dir_ += other.sum_view_dir_;

    ts_create_ = std::min(ts_create_, other.ts_create_);
    ts_update_ = std::max(ts_update_, other.ts_update_);
    dirty_ = true;
}

void Surfel::recompute() {
    if (!dirty_) return;
    if (W_ < 1e-8f || count_ == 0) {
        dirty_ = false;
        return;
    }

    const float inv_W = 1.0f / W_;
    mean_ = S1_ * inv_W;
    covariance_ = S2_ * inv_W - mean_ * mean_.transpose();
    covariance_ = 0.5f * (covariance_ + covariance_.transpose()); // ensure symmetry

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance_);
    if (solver.info() != Eigen::Success) {
        dirty_ = false;
        return;
    }

    eigenvalues_ = solver.eigenvalues().reverse().cwiseMax(0.0f); // descending order (l1 > l2 > l3)
    eigenvectors_ = solver.eigenvectors().rowwise().reverse();

    normal_ = eigenvectors_.col(2);

    if (sum_view_dir_.squaredNorm() > 1e-8f) {
        if (normal_.dot(sum_view_dir_) < 0.0f) {
            normal_ = -normal_;
            eigenvectors_.col(2) = -eigenvectors_.col(2);
        }
    }

    dirty_ = false;
}

Eigen::Vector3f Surfel::world_mean(float voxel_size) const {
    return Eigen::Vector3f(
        (static_cast<float>(key_.x) + 0.5f) * voxel_size,
        (static_cast<float>(key_.y) + 0.5f) * voxel_size,
        (static_cast<float>(key_.z) + 0.5f) * voxel_size
    ) + mean_;
}

float Surfel::planarity() const {
    if (eigenvalues_(0) < 1e-10f) return 0.0f;
    return (eigenvalues_(1) - eigenvalues_(2)) / eigenvalues_(0);
}

float Surfel::linearity() const {
    if (eigenvalues_(0) < 1e-10f) return 0.0f;
    return (eigenvalues_(0) - eigenvalues_(1)) / eigenvalues_(0);
}

float Surfel::sphericity() const {
    if (eigenvalues_(0) < 1e-10f) return 0.0f;
    return eigenvalues_(2) / eigenvalues_(0);
}

float Surfel::priority(int64_t now, size_t min_points_mature) const {
    const float maturity = std::min(
        static_cast<float>(count_) / static_cast<float>(min_points_mature), 1.0f);
    
    const float quality = planarity();
    const float age_sec = static_cast<float>(std::max(now - ts_update_, int64_t{0})) * NS_TO_SEC;
    const float recency = 1.0f / (1.0f + 0.2f * age_sec);
    return 0.4f * maturity + 0.4f * quality + 0.2f * recency;
}

}