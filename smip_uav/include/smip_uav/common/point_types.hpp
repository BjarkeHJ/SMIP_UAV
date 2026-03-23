#ifndef POINTNORMAL_HPP_
#define POINTNORMAL_HPP_

struct PointXYZ {
    float px, py, pz;
};

struct PointNormal {
    float px, py, pz; // Point position
    float nx, ny, nz; // Point normal
    float w; // Measurement weight
};

#endif