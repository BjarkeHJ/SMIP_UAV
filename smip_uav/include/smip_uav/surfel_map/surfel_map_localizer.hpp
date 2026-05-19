#ifndef SURFEL_MAP_LOCALIZER_HPP
#define SURFEL_MAP_LOCALIZER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "common/point_types.hpp"
#include "surfel_map/surfel_map.hpp"

namespace smip_uav {

class SurfelMapLocalizer {
public:
    struct Config {
        // Correspondences
        float corr_normal_cos{0.85f};
        float corr_mahal_sq{5.99f};
        float corr_max_range{0.3f};
        // Minimum sigma_n [m] applied before computing solver weights and whitened
        // residuals. Prevents sub-centimetre surfel covariances from inflating
        // avg_r2 and killing confidence on valid alignments.
        float sigma_n_floor{0.025f};
        bool only_converged{false};

        // Solver
        size_t max_iters{30};
        float step_norm_tol{1e-5f};
        // Exit when relative residual drop between iterations is below this.
        // Keep small (e.g. 1e-4) so ICP only stops when truly converged.
        // WARNING: values >= 0.01 cause premature exit after the first improving step.
        float residual_rel_tol{1e-4f};
        float tikhonov{1e-8f};

        // Huber loss
        float huber_k{2.0f};

        // Truncated pseudoinverse: eigenvectors with eigenvalue < ratio*max_eval are
        // excluded from the dx solution. Suppresses noise in geometrically unobservable
        // DOFs (e.g. roll when looking at a flat wall) without affecting observed DOFs.
        float obs_eigenvalue_ratio{0.01f};

        // Per-iteration step bound (mixed SE3 units: m + rad).
        // Small values stabilise convergence; large values risk overshooting when
        // early-iteration correspondences are noisy.
        float max_step_norm{0.15f};

        // Plausibility gate (Gate 2): reject if correction is physically implausible even
        // for a converged ICP. Gate 1 (avg_r2 quality check) already filters diverged
        // ICP before this is reached, so these limits can be generous enough to survive
        // fast yaw manoeuvres without rejecting valid large corrections.
        float max_correction_trans{0.5f};  // [m]
        float max_correction_rot{0.5f};    // [rad] ~28 deg

        // Confidence gating
        size_t min_inliers{30};
        float min_inlier_ratio{0.3f};
        // avg(r²/sigma_n²) above which c_resid → 0. Linearly maps [1, avg_r2_reject] to [1, 0].
        float avg_r2_reject{12.0f};
    };

    struct Result {
        Eigen::Isometry3f pose{Eigen::Isometry3f::Identity()}; // refined T_map_sensor
        bool localized{false}; // false -> fallback to prior
        float confidence{0.0f}; // [0,1] for slerp gain
        size_t iters{0};
        size_t inliers{0};
        size_t total_input{0};
        float final_residual{std::numeric_limits<float>::infinity()};
        float cond_number{0.0f}; // smallest/largers eval of final H
    };

    SurfelMapLocalizer() = default;
    explicit SurfelMapLocalizer(const Config& cfg, const SurfelMap& map);

    Result localize(const std::vector<FrameSurfel>& surfels_sensor, const Eigen::Isometry3f& T_prior);

private:
    struct Correspondence {
        const MapSurfel* map_surfel; // points into SurfelMap's voxel grid
        size_t frame_idx; // index into surfels_sensor
        float sigma_n;
    };

    size_t find_correspondences(const std::vector<FrameSurfel>& surfels_sensor, 
                                const Eigen::Isometry3f& T_ms, 
                                std::vector<Correspondence>& out);
    
    bool solve_step(const std::vector<FrameSurfel>& surfels_sensor,
                    const std::vector<Correspondence>& corrs,
                    const Eigen::Isometry3f& T_ms,
                    Eigen::Matrix<float,6,1>& dx_out,
                    float& residual_out,
                    float& cond_out);

    Config cfg_;
    const SurfelMap* map_{nullptr};
};

} // namespace smip_uav

#endif