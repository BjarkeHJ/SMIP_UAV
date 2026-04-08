#include "surfel_map/voxel_grid.hpp"

namespace smip_uav {

// VOXEL
MapSurfel* Voxel::try_add(const MapSurfel& s) {
    if (full()) return nullptr;
    surfels[count] = s;
    return &surfels[count++];
}

bool Voxel::remove_at(uint8_t idx) {
    if (idx >= count) return false;
    if (idx != count - 1) {
        surfels[idx] = std::move(surfels[count-1]); // swap with last element to keep array packed
    }
    --count; // decrement count to never check removed
    return true;
}

// VOXELGRID
VoxelGrid::VoxelGrid(const Config& cfg) : config_(cfg), inv_voxel_size_(1.0f / cfg.voxel_size) {
    assert(cfg.voxel_size > 0.0f && "Voxel size must be positive");
    voxels_.reserve(cfg.initial_bucket_count);
    voxels_.max_load_factor(cfg.max_load_factor);
}

VoxelKey VoxelGrid::to_key(float x, float y, float z) const {
    return {
        static_cast<int32_t>(std::floor(x * inv_voxel_size_)),
        static_cast<int32_t>(std::floor(y * inv_voxel_size_)),
        static_cast<int32_t>(std::floor(z * inv_voxel_size_))
    };
}

VoxelKey VoxelGrid::to_key(const Eigen::Vector3f& p) const {
    return to_key(p.x(), p.y(), p.z());
}

Eigen::Vector3f VoxelGrid::voxel_center(const VoxelKey& key) const {
    const float vs = config_.voxel_size;
    return {
        (static_cast<float>(key.x) + 0.5f) * vs,
        (static_cast<float>(key.y) + 0.5f) * vs,
        (static_cast<float>(key.z) + 0.5f) * vs
    };
}

Voxel& VoxelGrid::get_or_create(const VoxelKey& key) {
    return voxels_[key]; // operator[] default-constructs a Voxel if absent (count=0, empty)
}

Voxel* VoxelGrid::get(const VoxelKey& key) {
    auto it = voxels_.find(key);
    return it != voxels_.end() ? &it->second : nullptr;
}

const Voxel* VoxelGrid::get(const VoxelKey& key) const {
    auto it = voxels_.find(key);
    return it != voxels_.end() ? &it->second : nullptr;
}

bool VoxelGrid::remove(const VoxelKey& key) {
    return voxels_.erase(key) > 0;
}

void VoxelGrid::for_each_nb6(const VoxelKey& c, const std::function<void(const VoxelKey&, Voxel&)>& fn) {
    static constexpr int32_t offsets[6][3] = {
        {-1,0,0}, {1,0,0}, {0,-1,0}, {0,1,0}, {0,0,-1}, {0,0,1}
    };
    for (const auto& o : offsets) {
        VoxelKey nk{c.x + o[0], c.y + o[1], c.z + o[2]};
        if (auto* v = get(nk)) {
            fn(nk, *v);
        }
    }
}

void VoxelGrid::for_each_nb26(const VoxelKey& c, const std::function<void(const VoxelKey&, Voxel&)>& fn) {
    for (int32_t dx = -1; dx <= 1; ++dx) {
        for (int32_t dy = -1; dy <= 1; ++dy) {
            for (int32_t dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) continue;
                VoxelKey nk{c.x + dx, c.y + dy, c.z + dz};
                if (auto* v = get(nk)) {
                    fn(nk, *v);
                }
            }
        }
    }
}

size_t VoxelGrid::total_surfel_count() const {
    size_t total = 0;
    for (const auto& [key, voxel] : voxels_) {
        total += voxel.count;
    }
    return total;
}

} // smip_uav