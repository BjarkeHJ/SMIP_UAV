// #ifndef VOXEL_GRID_
// #define VOXEL_GRID_

// #include <unordered_map>
// #include "surfel_map/surfel.hpp"

// namespace smip_uav {

// struct Voxel {
//     static constexpr uint8_t MAX_NUM_SURFELS_PER_VOXLE = 4;

//     std::array<Surfel, MAX_NUM_SURFELS_PER_VOXLE> surfels;
//     uint8_t count{0};

//     // Try to add surfel: Return pointer to added surfel if added - nullptr if full/unable to add. 
//     Surfel* try_add(const Surfel& s); 
    
//     // Remove surfel at index. Return false if index is invalid
//     bool remove_at(uint8_t idx);
    
//     bool full() const { return count >= MAX_NUM_SURFELS_PER_VOXLE; }
//     bool empty() const { return  count == 0; }

//     // Iterable ranger over active surfels
//     Surfel* begin() { return surfels.data(); }
//     Surfel* end() { return surfels.data() + count; }
//     const Surfel* begin() const { return surfels.data(); }
//     const Surfel* end() const { return surfels.data() + count; }
// };

// class VoxelGrid {
// public:
//     using VoxelMap = std::unordered_map<VoxelKey, Voxel, VoxelKeyHash>;
//     using iter_v = VoxelMap::iterator;
//     using const_iter_v = VoxelMap::const_iterator;

//     struct Config {
//         float voxel_size{0.2f};
//         size_t initial_bucket_count{10000};
//         float max_load_factor{0.75f};
//     };

//     VoxelGrid() = default;
//     explicit VoxelGrid(const Config& config);

//     // Spatial Hashing
//     VoxelKey to_key(float x, float y, float z) const;
//     VoxelKey to_key(const Eigen::Vector3f& p) const;

//     // World-frame center of a voxel-cell
//     Eigen::Vector3f voxel_center(const VoxelKey& key) const;

//     // Lookup
//     Voxel& get_or_create(const VoxelKey& key); // get if exist - create if not
//     Voxel* get(const VoxelKey& key);
//     const Voxel* get(const VoxelKey& key) const;

//     // Removal
//     bool remove(const VoxelKey& key);

//     // Neighbor access: Calls fn(VoxelKey, Voxel&) for each existing neighboring voxel (nb6 or nb26 adjacencies)
//     void for_each_nb6(const VoxelKey& center, const std::function<void(const VoxelKey&, Voxel&)>& fn);
//     void for_each_nb26(const VoxelKey& center, const std::function<void(const VoxelKey&, Voxel&)>& fn);

//     // Iteration 
//     iter_v begin() { return voxels_.begin(); }
//     const_iter_v begin() const { return voxels_.begin(); }
//     iter_v end() { return voxels_.end(); }
//     const_iter_v end() const { return voxels_.end(); }

//     // Size
//     size_t size() const { return voxels_.size(); }
//     bool empty() const { return voxels_.empty(); }
//     void clear() { voxels_.clear(); }

//     float voxel_size() const { return config_.voxel_size; }
//     float voxel_inv_size() const { return inv_voxel_size_; }

//     // Stats
//     size_t total_surfel_count() const;

// private:
//     VoxelMap voxels_;
//     Config config_;
//     float inv_voxel_size_{0};
// }; 


// }; // smip_uav


// #endif