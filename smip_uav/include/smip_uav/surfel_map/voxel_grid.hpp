#ifndef VOXEL_GRID_
#define VOXEL_GRID_

#include <unordered_map>
#include <common/point_types.hpp>

namespace smip_uav {

struct VoxelKey {
    int32_t x,y,z;
    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
    bool operator!=(const VoxelKey& other) const {
        return !(*this == other);
    }
    bool operator<(const VoxelKey& other) const {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z; 
    }
};

struct VoxelKeyHash {
    size_t operator()(const VoxelKey& k) const {
        // spread bits of each coordinate then interleave
        auto spread = [](uint32_t v) -> uint64_t {
            uint64_t x = v;
            x = (x | x << 32) & 0x1f00000000ffff;
            x = (x | x << 16) & 0x1f0000ff0000ff;
            x = (x | x << 8)  & 0x100f00f00f00f00f;
            x = (x | x << 4)  & 0x10c30c30c30c30c3;
            x = (x | x << 2)  & 0x1249249249249249;
            return x;
        };
        return spread(k.x) | (spread(k.y) << 1) | (spread(k.z) << 2);
    }
};

struct Voxel {
    static constexpr uint8_t MAX_NUM_SURFELS_PER_VOXLE = 10;

    std::array<MapSurfel, MAX_NUM_SURFELS_PER_VOXLE> surfels;
    uint8_t count{0};

    // Try to add surfel: Return pointer to added surfel if added - nullptr if full/unable to add. 
    MapSurfel* try_add(const MapSurfel& s); 
    
    // Remove surfel at index. Return false if index is invalid
    bool remove_at(uint8_t idx);
    
    bool full() const { return count >= MAX_NUM_SURFELS_PER_VOXLE; }
    bool empty() const { return  count == 0; }

    // Iterable ranger over active surfels
    MapSurfel* begin() { return surfels.data(); }
    MapSurfel* end() { return surfels.data() + count; }
    const MapSurfel* begin() const { return surfels.data(); }
    const MapSurfel* end() const { return surfels.data() + count; }
};

class VoxelGrid {
public:
    using VoxelMap = std::unordered_map<VoxelKey, Voxel, VoxelKeyHash>;
    using iter_v = VoxelMap::iterator;
    using const_iter_v = VoxelMap::const_iterator;

    struct Config {
        float voxel_size{0.3f};
        size_t initial_bucket_count{10000};
        float max_load_factor{0.75f};
    };

    VoxelGrid() = default;
    explicit VoxelGrid(const Config& config);

    // Spatial Hashing
    VoxelKey to_key(float x, float y, float z) const;
    VoxelKey to_key(const Eigen::Vector3f& p) const;

    // World-frame center of a voxel-cell
    Eigen::Vector3f voxel_center(const VoxelKey& key) const;

    // Lookup
    Voxel& get_or_create(const VoxelKey& key); // get if exist - create if not
    Voxel* get(const VoxelKey& key);
    const Voxel* get(const VoxelKey& key) const;

    // Removal
    bool remove(const VoxelKey& key);

    // Neighbor access: Calls fn(VoxelKey, Voxel&) for each existing neighboring voxel (nb6 or nb26 adjacencies)
    template<typename Fn>
    void for_each_nb6(const VoxelKey& c, Fn&& fn) {
        static constexpr int32_t offsets[6][3] = {
            {-1,0,0}, {1,0,0}, {0,-1,0}, {0,1,0}, {0,0,-1}, {0,0,1}
        };
        for (const auto& o : offsets) {
            VoxelKey nk{c.x + o[0], c.y + o[1], c.z + o[2]};
            if (auto* v = get(nk)) fn(nk, *v);
        }
    }

    template<typename Fn>
    void for_each_nb26(const VoxelKey& c, Fn&& fn) {
        for (int32_t dx = -1; dx <= 1; ++dx)
            for (int32_t dy = -1; dy <= 1; ++dy)
                for (int32_t dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    VoxelKey nk{c.x + dx, c.y + dy, c.z + dz};
                    if (auto* v = get(nk)) fn(nk, *v);
                }
    }

    // Iteration 
    iter_v begin() { return voxels_.begin(); }
    const_iter_v begin() const { return voxels_.begin(); }
    iter_v end() { return voxels_.end(); }
    const_iter_v end() const { return voxels_.end(); }

    // Size
    size_t size() const { return voxels_.size(); }
    bool empty() const { return voxels_.empty(); }
    void clear() { voxels_.clear(); }

    float voxel_size() const { return config_.voxel_size; }
    float voxel_inv_size() const { return inv_voxel_size_; }

    // Stats
    size_t total_surfel_count() const;

private:
    VoxelMap voxels_;
    Config config_;
    float inv_voxel_size_{0};
}; 


}; // smip_uav


#endif