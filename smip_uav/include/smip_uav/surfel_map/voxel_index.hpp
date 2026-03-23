#ifndef VOXEL_INDEX_
#define VOXEL_INDEX_

#include "surfel_map/surfel.hpp"

namespace smip_uav {

class VoxelIndex {
public:
    struct Config {
        float voxel_size{0.3f};
        size_t initial_bucket_count{10000};
        float max_load_factor{0.75f};
    };

    VoxelIndex();

private:


}; 


}; // smip_uav


#endif