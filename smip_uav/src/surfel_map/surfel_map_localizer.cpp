#include "surfel_map/surfel_map_localizer.hpp"

namespace smip_uav {

namespace {
// Full 27-neighbourhood (all combinations of {-1,0,1}^3).
// Compared with the 7-face neighbourhood, this also covers edge- and corner-adjacent
// voxels, which matters when a frame surfel projects near a voxel boundary and its
// best map match sits in a diagonal neighbour up to sqrt(3)*voxel_size away.
constexpr int32_t kNb27[27][3] = {
    {-1,-1,-1}, {-1,-1, 0}, {-1,-1, 1},
    {-1, 0,-1}, {-1, 0, 0}, {-1, 0, 1},
    {-1, 1,-1}, {-1, 1, 0}, {-1, 1, 1},
    { 0,-1,-1}, { 0,-1, 0}, { 0,-1, 1},
    { 0, 0,-1}, { 0, 0, 0}, { 0, 0, 1},
    { 0, 1,-1}, { 0, 1, 0}, { 0, 1, 1},
    { 1,-1,-1}, { 1,-1, 0}, { 1,-1, 1},
    { 1, 0,-1}, { 1, 0, 0}, { 1, 0, 1},
    { 1, 1,-1}, { 1, 1, 0}, { 1, 1, 1},
};
}

SurfelMapLocalizer::SurfelMapLocalizer(const Config& cfg, const SurfelMap& map) : cfg_(cfg), map_(&map) {}

SurfelMapLocalizer::Result SurfelMapLocalizer::localize(
    const std::vector<FrameSurfel>& surfels_sensor, 
    const Eigen::Isometry3f& prior) 
{
    Result res;
    res.pose = prior;
    res.total_input = surfels_sensor.size();
    
    if (!map_ || surfels_sensor.empty()) return res;

    std::vector<Correspondence> corrs;
    corrs.reserve(surfels_sensor.size());

    float prev_residual = std::numeric_limits<float>::infinity();
    float last_cond = 0.0f;
    size_t last_inliers = 0;

    for (size_t iter = 0; iter < cfg_.max_iters; ++iter) {
        const size_t n_corrs = find_correspondences(surfels_sensor, res.pose, corrs);
        if (n_corrs < cfg_.min_inliers) {
            break; // Too few correspondences to solve -> bail
        }

        last_inliers = n_corrs;

        Eigen::Matrix<float,6,1> dx;
        float residual = 0.0f;
        float cond = 0.0f;
        if (!solve_step(surfels_sensor, corrs, res.pose, dx, residual, cond)) {
            break;
        }

        last_cond = cond;

        // Clamp step magnitude to prevent ICP from walking into a false basin
        const float dx_norm = dx.norm();
        if (dx_norm > cfg_.max_step_norm) {
            dx *= (cfg_.max_step_norm / dx_norm);
        }

        // Apply SE3 left update: T_new = Exp(dx) * T_cur
        const Eigen::Vector3f t = dx.head<3>();
        const Eigen::Vector3f om = dx.tail<3>();
        const float angle = om.norm();
        Eigen::Isometry3f dT = Eigen::Isometry3f::Identity();
        if (angle > 1e-8f) {
            dT.linear() = Eigen::AngleAxisf(angle, om / angle).toRotationMatrix();
        }
        dT.translation() = t;
        res.pose = dT * res.pose;

        res.iters = iter + 1;
        res.final_residual = residual;

        // Convergence checks
        if (dx.norm() < cfg_.step_norm_tol) break;
        
        if (prev_residual < std::numeric_limits<float>::infinity()) {
            const float rel = (prev_residual - residual) / (prev_residual + 1e-10f);
            if (rel < cfg_.residual_rel_tol && rel >= 0.0f) break;
        }
        prev_residual = residual;
    }

    res.inliers = last_inliers;
    res.cond_number = last_cond;

    // avg(r²/sigma_n²) — computed once, used in both gates below.
    const float avg_r2 = res.inliers > 0
        ? res.final_residual / static_cast<float>(res.inliers) : 1e9f;

    // Gate 1 — quality: ICP clearly diverged (large residual regardless of correction size).
    // Rejects false minima where ICP "converged" to a wrong alignment.
    if (avg_r2 >= cfg_.avg_r2_reject) {
        return res; // confidence stays 0, localized stays false
    }

    // Gate 2 — plausibility: correction is physically implausible even for a converged ICP.
    // Catches gross failures in symmetric/degenerate geometry (e.g. long corridor).
    // Limits are set generously to allow large corrections during fast yaw manoeuvres;
    // gate 1 already ensures ICP quality is acceptable before this is reached.
    {
        const Eigen::Isometry3f dT = res.pose * prior.inverse();
        const float d_trans = dT.translation().norm();
        const float d_rot   = Eigen::AngleAxisf(dT.linear()).angle();
        if (d_trans > cfg_.max_correction_trans || d_rot > cfg_.max_correction_rot) {
            res.pose = prior;
            return res;
        }
    }

    const float inlier_ratio = res.total_input > 0
        ? static_cast<float>(res.inliers) / static_cast<float>(res.total_input)
        : 0.0f;

    const float c_inlier = std::clamp(
        (inlier_ratio - cfg_.min_inlier_ratio) / (1.0f - cfg_.min_inlier_ratio),
        0.0f, 1.0f);

    const float c_resid = std::clamp(
        1.0f - (avg_r2 - 1.0f) / (cfg_.avg_r2_reject - 1.0f),
        0.0f, 1.0f);

    res.confidence = c_inlier * c_resid;
    res.localized = (res.inliers >= cfg_.min_inliers) && (res.confidence > 0.0f);
        
    return res;
}

size_t SurfelMapLocalizer::find_correspondences(
    const std::vector<FrameSurfel>& surfels_sensor,
    const Eigen::Isometry3f& T_ms,
    std::vector<Correspondence>& out) 
{
    out.clear();
    out.reserve(surfels_sensor.size());
    
    const auto& grid = map_->grid();
    const float inv_vs = grid.voxel_inv_size();
    const float max_range_sq = cfg_.corr_max_range * cfg_.corr_max_range;
    const Eigen::Matrix3f R = T_ms.rotation();

    for (size_t i = 0; i < surfels_sensor.size(); ++i) {
        const FrameSurfel& fs = surfels_sensor[i];

        // transform frame surfel into map frame using current pose estimate
        const Eigen::Vector3f mu_w = T_ms * fs.centroid;
        const Eigen::Vector3f n_w = R * fs.normal;
        const Eigen::Matrix3f S_f_w = R * fs.R * R.transpose();

        const VoxelKey kc{
            static_cast<int32_t>(std::floor(mu_w.x() * inv_vs)),
            static_cast<int32_t>(std::floor(mu_w.y() * inv_vs)),
            static_cast<int32_t>(std::floor(mu_w.z() * inv_vs))
        };

        const MapSurfel* best = nullptr;
        float best_score = std::numeric_limits<float>::max();
        float best_sigma_n = 0.0f;

        for (const auto& o : kNb27) {
            const VoxelKey k{kc.x + o[0], kc.y + o[1], kc.z + o[2]};
            const Voxel* v = grid.get(k);
            if (!v) continue;

            for (const MapSurfel& ms : *v) {
                if (cfg_.only_converged && !ms.converged) continue;

                // Normal Alignment gate
                const float cos_n = n_w.dot(ms.normal);
                if (cos_n < cfg_.corr_normal_cos) continue;

                // Range range
                const Eigen::Vector3f d = mu_w - ms.mu;
                if (d.squaredNorm() > max_range_sq) continue;

                // Tangential Mahalanobis
                const Eigen::Vector3f n_avg = (n_w + ms.normal).normalized();
                const Eigen::Vector3f d_tan = d - d.dot(n_avg) * n_avg;
                const Eigen::Matrix3f S = ms.sigma + S_f_w;
                const float d2 = d_tan.dot(S.ldlt().solve(d_tan));
                if (!std::isfinite(d2) || d2 < 0.0f) continue;
                if (d2 >= cfg_.corr_mahal_sq) continue;

                // Score: Combine normla alignment and tangential distance (primarily d2)
                const float score = d2 - 0.1*cos_n;
                if (score < best_score) {
                    best = &ms;
                    best_score = score;

                    // Pre-compute sigma_n along normal axis for Huber scaling.
                    // Floor at sigma_n_floor so tight map surfels do not
                    // over-inflate the whitened residual and kill confidence.
                    const float sigma2_n = ms.normal.dot((ms.sigma + S_f_w) * ms.normal);
                    best_sigma_n = std::max(
                        std::sqrt(std::max(sigma2_n, 1e-10f)),
                        cfg_.sigma_n_floor);
                }
            }
        }

        if (best) {
            out.push_back({best, i, best_sigma_n});
        }
    }
    return out.size();
}

bool SurfelMapLocalizer::solve_step(
    const std::vector<FrameSurfel>& surfels_sensor,
    const std::vector<Correspondence>& corrs,
    const Eigen::Isometry3f& T_ms,
    Eigen::Matrix<float,6,1>& dx_out,
    float& residual_out,
    float& cond_out) 
{
    Eigen::Matrix<float,6,6> H = Eigen::Matrix<float,6,6>::Zero();
    Eigen::Matrix<float,6,1> b = Eigen::Matrix<float,6,1>::Zero();
    float r_sq_sum = 0.0f;

    for (const auto& c : corrs) {
        const FrameSurfel& fs = surfels_sensor[c.frame_idx];
        const MapSurfel& ms = *c.map_surfel;
        
        // Transform frame surfels centroid to map at current pose
        const Eigen::Vector3f mu_w = T_ms * fs.centroid;

        // Point-to-plane residual
        const Eigen::Vector3f& n_j = ms.normal;
        const float r = n_j.dot(mu_w - ms.mu);

        // Information weight from anisotropic combined covariance along normal
        const float sigma_n = c.sigma_n;
        const float w_info = 1.0f / (sigma_n * sigma_n);

        // Huber robust loss
        const float r_whitened = std::abs(r) / sigma_n;
        const float w_robust = (r_whitened <= cfg_.huber_k) ? 1.0f : cfg_.huber_k / r_whitened;
        const float w = w_info * w_robust;

        // Jacobian for left SE3 pertubation
        Eigen::Matrix<float,1,6> J;
        J.head<3>() = n_j.transpose();
        J.tail<3>() = mu_w.cross(n_j).transpose();

        H.noalias() += w * J.transpose() * J;
        b.noalias() += w * J.transpose() * r;
        r_sq_sum += w_info * r * r; // un-robustified for convergence check
    }

    // Tikhonov for rank-deficient geometry (e.g. single-plane lock)
    H.diagonal().array() += cfg_.tikhonov;

    // Conditioning diagnostic
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float,6,6>> eig(H);
    if (eig.info() != Eigen::Success) return false;
    const auto& evals = eig.eigenvalues();
    const auto& evecs = eig.eigenvectors();

    Eigen::Matrix<float,6,1> b_eig = evecs.transpose() * (-b);

    Eigen::Matrix<float,6,1> dx_eig;
    const float max_eval = evals.maxCoeff();
    float observable_dims = 0;
    for (int i = 0; i < 6; ++i) {
        const float ratio = evals(i) / max_eval;

        if (ratio < cfg_.obs_eigenvalue_ratio) {
            dx_eig(i) = 0.0f; // suppres underconstrained DOF
        }
        else {
            dx_eig(i) = b_eig(i) / (evals(i) + 1e-8f);
            observable_dims += 1.0f;
        }
    }
    dx_out = evecs * dx_eig;

    cond_out = static_cast<float>(observable_dims) / 6.0f;

    if (!dx_out.allFinite()) return false;

    residual_out = r_sq_sum;
    return true;
}


} // namespace smip_uav