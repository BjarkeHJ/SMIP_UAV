#ifndef SMIP_TYPES_HPP_
#define SMIP_TYPES_HPP_

namespace smip_uav {

struct PointXYZ {
    float px, py, pz;
};

struct Normal {
    float nx, ny, nz;
};

struct PointNormalWeight {
    PointXYZ p;
    Normal n;
    float w;
};


} // namespace smip_uav

#endif