#ifndef VOXEL_GRID_
#define VOXEL_GRID_

#include <unordered_map>
#include <optional>
#include "surfel_map/surfel.hpp"

namespace smip_uav {

class VoxelGrid {
public:
    using VoxelMap = std::unordered_map<VoxelKey, Surfel, VoxelKeyHash>;
    using iter_v = VoxelMap::iterator;
    using const_iter_v = VoxelMap::const_iterator;

    struct Config {
        float voxel_size{0.3f};
        size_t initial_bucket_count{10000};
        float max_load_factor{0.75f};
    };

    VoxelGrid() = default;
    explicit VoxelGrid(const Config& config);

    Surfel& get_or_create(const VoxelKey& key); // get if exist - create if not
    std::optional<std::reference_wrapper<Surfel>> get(const VoxelKey& key);
    std::optional<std::reference_wrapper<const Surfel>> get(const VoxelKey& key) const;

    
    bool remove(const VoxelKey& key);

    iter_v begin() { return voxels_.begin(); }
    const_iter_v begin() const { return voxels_.begin(); }
    iter_v end() { return voxels_.end(); }
    const_iter_v end() const { return voxels_.end(); }

    size_t size() const { return voxels_.size(); }
    bool empty() const { return voxels_.empty(); }

private:
    VoxelMap voxels_;
    Config config_;
}; 


}; // smip_uav


#endif