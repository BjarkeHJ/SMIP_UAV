#ifndef SURFEL_MAP_HPP_
#define SURFEL_MAP_HPP_

#include <iostream>
#include <memory>
#include <unordered_set>
#include "surfel_map/frame_builder.hpp"
#include "surfel_map/frame_processor.hpp"
#include "surfel_map/voxel_grid.hpp"

namespace smip_uav {

class SurfelMap {
public:
    struct Config {
        VoxelGrid::Config grid_config;

        // GMM: E-step
        float pi_spawn{0.005f}; // spawn prior - higher = easier spawn new surfels
        float spawn_residual{0.75f}; // r_new threshold to spawn new surfel

        // M-step
        float gamma_forget{0.99f}; // Forgetting factor

        // Normal alignment - shared angular scale for E-step and merge
        float normal_sigma{static_cast<float>(M_PI) / 8.0f}; // std-dev of normal Gaussian (rad)
        float merge_normal_k{0.5f}; // merge threshold at k*sigma — must be < E-step 1-sigma

        // Merge
        float merge_mahal_sq{3.0f}; // mahalanobis threshold for merging
        uint32_t merge_interval{5}; // frames between merge passes
    };

    SurfelMap() = default;
    explicit SurfelMap(const Config& cfg);

    void update_map(const std::vector<FrameSurfel>& frame_surfels, const Eigen::Isometry3f& pose, int64_t timestamp_ns);

    const std::vector<MapSurfel*>& get_all_surfels();
    std::vector<MapSurfel*> get_updated_surfels(); // reset after each call

    size_t surfel_count() const { return grid_->total_surfel_count(); }
    const VoxelGrid& grid() const { return *grid_; }

    // Incremental viz delta tracking
    const std::unordered_set<uint32_t>& deleted_ids() const { return deleted_ids_; }

private:
    // One entry in the responsibility vector for a single frame surfel
    struct RespEntry {
        MapSurfel* component;
        float log_r_tilde; // unnormalized log-responsibility
        float r; // normalized responsibility [0,1]
    };

    // Per component accumulator for one frame's E/M step
    struct ComponentAccum {
        float delta_W{0.0f};
        Eigen::Vector3f delta_S1{Eigen::Vector3f::Zero()};
        Eigen::Matrix3f delta_S2{Eigen::Matrix3f::Zero()};
    };

    void integrate(const std::vector<FrameSurfel>& frame_surfels, const Eigen::Isometry3f& pose, int64_t timestamp_ns);
    float compute_responsibilities(const FrameSurfel& fs_w, std::vector<RespEntry>& resp_out);
    void spawn(const FrameSurfel& fs_w, int64_t timestamp_ns);
    void merge();

    // Helpers
    FrameSurfel transform_surfel_to_world(const FrameSurfel& fs, const Eigen::Isometry3f& pose) const;
    void clear_deltas() { updated_ids_.clear(); deleted_ids_.clear(); };

    // Voxel Grid
    std::unique_ptr<VoxelGrid> grid_;

    // Buffers
    std::vector<MapSurfel*> surfel_cache_;
    bool cache_dirty_{false};

    std::unordered_map<MapSurfel*, ComponentAccum> accums_;
    std::vector<FrameSurfel> spawn_candidates_;
    std::vector<RespEntry> resp_;

    // Incremental deltas
    std::unordered_set<uint32_t> updated_ids_;   // new or updated since last get_updated_surfels() call
    std::unordered_set<uint32_t> deleted_ids_; // removed since last get_updated_surfels() call

    // Config
    Config cfg_;

    // Constants
    float log_2pi_1_5_{0.0f};    // 1.5 * log(2pi)
    float log_lambda_new_{0.0f}; // log(pi_spawn / V_voxel)
    float inv_2_sigma_n_sq_{0.0f}; // 1 / (2 * sigma_n^2) — E-step normal penalty scale
    float merge_normal_cos_{0.0f}; // cos(k * sigma_n) — merge hard gate, derived from normal_sigma

    // Counters
    uint32_t frame_count_{0};
    mutable int32_t next_id_{1};
};

} // smip_uav


#endif