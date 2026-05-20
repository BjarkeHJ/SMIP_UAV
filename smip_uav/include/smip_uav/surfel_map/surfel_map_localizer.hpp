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
        bool only_converged{false};
        float corr_normal_cos{0.85f};
        float corr_mahal_sq{5.99f};
        float corr_max_range{0.3f};
        float sigma_n_floor{0.025f};

        // Solver
        size_t max_iters{30};
        float step_norm_tol{1e-5f};
        float residual_rel_tol{1e-4f};
        float obs_eigenvalue_ratio{0.01f};

        // Huber loss
        float huber_k{2.0f};

        float max_step_norm{0.05f};
        float max_correction_trans{0.5f};  // [m]
        float max_correction_rot{0.5f};    // [rad] ~28 deg

        // Localization gating
        size_t min_inliers{10};
        float min_inlier_ratio{0.3f};
        float avg_r2_reject{12.0f};
    };

    struct Result {
        bool localized{false};
        Eigen::Isometry3f pose{Eigen::Isometry3f::Identity()};
        Eigen::Matrix<float,6,6> H_icp{Eigen::Matrix<float,6,6>::Zero()};
        size_t iters{0};
        size_t inliers{0};
        size_t total_input{0};
        float inlier_ratio{0.0f};
        float final_residual{std::numeric_limits<float>::infinity()};
        float cond_number{0.0f};
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
                    Eigen::Matrix<float,6,6>& H_out,
                    float& residual_out,
                    float& cond_out);

    Config cfg_;
    const SurfelMap* map_{nullptr};
};

} // namespace smip_uav

#endif