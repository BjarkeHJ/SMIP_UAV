#ifndef SMIP_FRAME_HPP_
#define SMIP_FRAME_HPP_

#include <cstdint>
#include <vector>
#include <limits>
#include <Eigen/Geometry>

#include "core/surfel.hpp"

namespace smip_uav {

struct PointNormal {
    Eigen::Vector3f p;
    Eigen::Vector3f n;
};

struct PointBuffer {
    std::vector<PointNormal> pointnormals;
    std::vector<float> weights;
    std::vector<float> ranges;
    std::vector<uint8_t> validities;
};

struct FrameMetadata {
    uint64_t frame_id;
    uint32_t width{0};
    uint32_t height{0};
    Eigen::Isometry3f T_sensor_world;
    int64_t stamp;
};

struct Frame {
    FrameMetadata meta;
    PointBuffer pixels;

    std::vector<Surfel> surfels;

    Frame() = default;
    Frame(const uint32_t width, const uint32_t height, const Eigen::Isometry3f& T_sensor_world, int64_t stamp) {
        meta.width = width;
        meta.height = height;
        meta.T_sensor_world = T_sensor_world;
        meta.stamp = stamp;
        const size_t n = static_cast<size_t>(width) * height;
        pixels.pointnormals.resize(n);
        pixels.weights.resize(n, 0.0f);
        pixels.ranges.resize(n, std::numeric_limits<float>::infinity());
        pixels.validities.resize(n, 0);
    }

    size_t size() const {
        return pixels.pointnormals.size();
    }
};


} // namespace smip_uav

#endif 