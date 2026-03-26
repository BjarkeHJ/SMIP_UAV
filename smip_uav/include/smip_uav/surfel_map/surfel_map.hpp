#ifndef SURFEL_MAP_HPP_
#define SURFEL_MAP_HPP_

#include "surfel_map/voxel_grid.hpp"

namespace smip_uav {

class SurfelMap {
public:
    // Config Struct
    struct SurfelMapConfig {
        
    };

    SurfelMap();
    explicit SurfelMap(const SurfelMapConfig& config);

    void update();


private:
    SurfelMapConfig config_;


    uint64_t ts_create_;
    uint64_t ts_update_;

};


} // smip_uav


#endif