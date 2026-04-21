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
        float pi_spawn{0.005f}; // spawn prior - higher = easier spawn new surfels
        float spawn_residual{0.75f}; // r_new threshold to spawn new surfel

        // Merge
        float merge_mahal_sq{3.0f}; // mahalanobis threshold for merging
        float merge_normal_cos{0.99f}; // ~18 deg normal alignment required for merge
        uint32_t merge_interval{5}; // frames between merge passes
    };

    SurfelMap() = default;
    explicit SurfelMap(const Config& cfg);

    void update(const std::vector<PointXYZ>& scan, const Eigen::Isometry3f& pose, int64_t timestamp_ns, std::vector<FrameSurfel>* frame_surfels_out=nullptr);

    const std::vector<MapSurfel*>& get_all_surfels();
    std::vector<MapSurfel*> get_updated_surfels(); // reset after each call

    size_t surfel_count() const { return grid_->total_surfel_count(); }
    const Frame& frame() const { return frame_; }

    // Per-pixel surfel labels from the last processed frame (-1 = unassigned)
    const std::vector<int32_t>& frame_labels() const { return processor_->labels(); }

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

    // Components
    std::unique_ptr<FrameBuilder> builder_;
    std::unique_ptr<FrameProcessor> processor_;
    std::unique_ptr<VoxelGrid> grid_;

    // Buffers
    Frame frame_;
    std::vector<MapSurfel*> surfel_cache_;
    bool cache_dirty_{false};

    // Incremental deltas
    std::unordered_set<uint32_t> updated_ids_;   // new or updated since last get_updated_surfels() call
    std::unordered_set<uint32_t> deleted_ids_; // removed since last get_updated_surfels() call

    // Config
    Config cfg_;

    // Constants
    float log_2pi_1_5_{0.0f}; // 1.5 * log(2pi)
    float log_lambda_new_{0.0f}; // log(pi_spawn / V_voxel)

    // Counters
    uint32_t frame_count_{0};
    mutable int32_t next_id_{1};
};

} // smip_uav


#endif