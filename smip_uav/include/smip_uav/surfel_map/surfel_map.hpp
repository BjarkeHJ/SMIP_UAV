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
        // ENTIRE CONFIG - Will branch out
        FrameBuilder::Config builder_config;
        FrameProcessor::Config processor_config;
        VoxelGrid::Config grid_config;

        // GMM: E-step
        // float normal_gate_cos{0.866f}; // hard normal gate filter (30 deg)
        float normal_gate_cos{0.95f}; // hard normal gate filter (30 deg)
        float pi_birth{0.005f}; // birth prior - higher = easier birth of new surfels
        float birth_residual{0.5f}; // r_new threshold to spawn new surfel

        // Competitive health
        float health_alpha{0.2f}; // EMA rate for health update
        float health_min{0.05f}; // prune if health < this
        uint32_t eval_min{5}; // min evaluations before pruning is allowed

        // Merge
        float merge_mahal_sq{3.0f}; // mahalanobis threshold for merging
        float merge_normal_cos{0.95f}; // ~18 deg normal alignment required for merge
        uint32_t merge_interval{10}; // frames between merge passes
    };

    SurfelMap() = default;
    explicit SurfelMap(const Config& cfg);

    void update(const std::vector<PointXYZ>& scan, const Eigen::Isometry3f& pose, int64_t timestamp_ns, std::vector<FrameSurfel>* frame_surfels_out=nullptr);

    const std::vector<MapSurfel*>& get_all_surfels();
    size_t surfel_count() const { return grid_->total_surfel_count(); }
    const Frame& frame() const { return frame_; }
    // Per-pixel surfel labels from the last processed frame (-1 = unassigned)
    const std::vector<int32_t>& frame_labels() const { return processor_->labels(); }

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
        float available_weight{0.0f}; // toatal w_k of candidates (for health)
    };

    void integrate(const std::vector<FrameSurfel>& frame_surfels, const Eigen::Isometry3f& pose, int64_t timestamp_ns);
    float compute_responsibilities(const FrameSurfel& fs_w, std::vector<RespEntry>& resp_out);
    void birth(const FrameSurfel& fs_w, int64_t timestamp_ns);
    void prune();
    void merge();

    // Helpers
    FrameSurfel transform_surfel_to_world(const FrameSurfel& fs, const Eigen::Isometry3f& pose) const;

    // Components
    std::unique_ptr<FrameBuilder> builder_;
    std::unique_ptr<FrameProcessor> processor_;
    std::unique_ptr<VoxelGrid> grid_;

    // Buffers
    Frame frame_;
    std::vector<MapSurfel*> surfel_cache_;
    bool cache_dirty_{false};

    // Config
    Config cfg_;

    // Constants
    float log_2pi_1_5_{0.0f}; // 1.5 * log(2pi)
    float log_lambda_new_{0.0f}; // log(pi_birth / V_voxel)
    float lambda_max_{0.0f}; // max surfel dimension

    // Counters
    uint32_t frame_count_{0};
    mutable int32_t next_id_{1};
};


} // smip_uav


#endif