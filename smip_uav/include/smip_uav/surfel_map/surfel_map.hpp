#ifndef SURFEL_MAP_HPP_
#define SURFEL_MAP_HPP_

#include "surfel_map/voxel_grid.hpp"

namespace smip_uav {

class SurfelMap {
public:
    // Config Struct
    struct Config {
        VoxelGrid::Config grid_config{};

        // Association
        float assoc_normal_cos_mature{0.94}; // 0.94 ~20deg
        float assoc_normal_cos_tentative{0.5}; //0.5 ~40deg
        float assoc_point_to_plane_max{0.05}; // meters
        
        // Lifecycle
        size_t surfel_min_points{25};
        size_t surfel_max_points{10000};
        
        // Creation
        float creation_min_normal_separaton_cos{0.85f}; // 0.85 ~32deg
        
        // Maintenance
        size_t maintanence_interval_N{10};
        float merge_normal_cos{0.90};
        float merge_point_to_plane_max{0.1f};
        float boundary_margin_ratio{0.2f};
        float tentative_timeout_sec{3.0f};
        float min_planarity{0.15f};
    };

    SurfelMap();
    explicit SurfelMap(const Config& cfg);

    void integrate_frame(const Frame& frame);

    // Finalized and maintained map (not in fragile state): What is open to the "public"
    bool has_map() const { return has_public_; }
    const VoxelGrid& map() const { return grid_public_; }
    std::vector<const Surfel*> surfels() const;

    size_t frame_count() const { return frame_count_; }
    size_t working_voxel_count() const { return grid_.size(); }
    size_t working_surfel_count() const { return grid_public_.total_surfel_count(); }

private:
    // Per-Point integration
    void associate_and_fuse(const PointNormal& pn, const Eigen::Vector3f& view_dir, uint64_t timestamp);
    Surfel* find_best_match(Voxel& voxel, const PointNormal& pn);
    void handle_new_surface(Voxel& voxel, const PointNormal& pn, const Eigen::Vector3f& view_dir, const VoxelKey& key, uint64_t timestamp);

    // Maintenance
    void run_maintenance(uint64_t timestamp);
    void recompute_dirty(Voxel& voxel);
    void merge_similar(Voxel& voxel);
    void merge_boundary_surfels(const VoxelKey& key, Voxel& voxel);
    void evict_stale(Voxel& voxel, uint64_t timestamp);
    void evict_low_quality(Voxel& voxel);

    void update_public(); // update public map representation

    // Helpers
    bool merge_ok(const Surfel& a, const Eigen::Vector3f& pos_a_world, const Surfel& b, const Eigen::Vector3f& pos_b_world) const;
    bool merge_ok_same_voxel(const Surfel& a, const Surfel& b, const VoxelKey& key) const;
    bool try_free_slot(Voxel& voxel, const VoxelKey& key, uint64_t timestamp);

    // State
    VoxelGrid grid_;
    VoxelGrid grid_public_;
    bool has_public_{false};
    Config config_;
    size_t frame_count_{0};
};


} // smip_uav


#endif