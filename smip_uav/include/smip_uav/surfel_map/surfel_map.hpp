#ifndef SURFEL_MAP_HPP_
#define SURFEL_MAP_HPP_

#include <iostream>
#include <memory>
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

        // Association
        // float normal_gate_cos{0.866f};   // cos(30°)
        float normal_gate_cos{0.9659f};   // cos(15)
        float mahal_gate_sq{11.345f};    // chi² 3-DOF 99%

        // Fusion
        float kappa{5.0f};             // tangential inflation
        float process_noise_rate{1e-7f}; // process noise rate to accomodate sligth drift of unobserved surfels
    };

    SurfelMap() = default;
    explicit SurfelMap(const Config& cfg);

    // Update SurfelMap with incoming Pointcloud + Pose (Option to retrieve surfels extracted from this frame)
    void update(const std::vector<PointXYZ>& scan, const Eigen::Isometry3f& pose, int64_t timestamp_ns, std::vector<FrameSurfel>* frame_surfels_out = nullptr);

    // Access
    const std::vector<MapSurfel*>& get_all_surfels();
    size_t surfel_count() const { return grid_->total_surfel_count(); }
    const Frame& frame() const { return frame_; }

private:
    // Integration
    void integrate(const std::vector<FrameSurfel>& frame_surfels, const Eigen::Isometry3f& pose, int64_t timestamp_ns);
    MapSurfel* find_association(const FrameSurfel& fs_w);

    // Helpers
    FrameSurfel transform_surfel_to_world(const FrameSurfel& fs_local, const Eigen::Isometry3f& pose) const;
    MapSurfel create_map_surfel(const FrameSurfel& fs_w, int64_t timestamp_ns);
    void fuse(MapSurfel& ms, const FrameSurfel& fs_w, int64_t timestamp_ns);

    // Components
    std::unique_ptr<FrameBuilder> builder_;
    std::unique_ptr<FrameProcessor> processor_;
    std::unique_ptr<VoxelGrid> grid_;

    // Buffer
    Frame frame_;
    std::vector<MapSurfel*> surfel_cache_;
    bool cache_dirty_{true};

    // Config
    Config cfg_;

    // id 
    static constexpr int32_t INVALID_SURFEL_ID{0};
    mutable int32_t next_id_{1}; 

};


} // smip_uav


#endif