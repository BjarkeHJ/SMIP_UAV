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
        bool only_converged{true};

        // Solver
        size_t max_iters{6};
        float step_norm_tol{1e-5f}; // convergence
        float residual_rel_tol{1e-4f}; // convergence 
        float tikhonov{1e-6f}; // H diagonal regularization

        // Huber loss
        float huber_k{2.0f};

        // Confidence gating
        size_t min_inliers{30};
        float min_inlier_ratio{0.3f};
        float min_cond_number{1e-4f}; // lambda_min(H) / lambda_max(H)
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