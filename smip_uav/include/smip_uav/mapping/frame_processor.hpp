#ifndef SMIP_FRAME_PROCESSOR_HPP_
#define SMIP_FRAME_PROCESSOR_HPP_

#include "core/frame.hpp"

namespace smip_uav {

enum class SensorRotation {
    DEG_0 = 1,
    DEG_90 = 2,
    DEG_180 = 3,
    DEG_270 = 4
};

class FrameProcessor {
public:
    struct Config {
        size_t tof_res_x{240}; // fov 106 deg
        size_t tof_res_y{180}; // fov 86 deg
        SensorRotation sensor_rotation{SensorRotation::DEG_0};
        float min_range{0.3f};
        float max_range{5.0f};
        float pixel_pitch{0.01071f}; //Radians per pixel -> DIRECTLY FROM SENSOR SPECS: 0.5(tan(hfov/2)/(resx/2) + tan(vfox/2)/resy/2)
        int ds_factor{1};
    };

    FrameProcessor() : FrameProcessor(FrameProcessor::Config{}) {}
    explicit FrameProcessor(const Config& config);

    void process(Frame& frame);

private:
    // Image Projection geometry (immutable after construction)
    struct ImageProjection {
        size_t W, H, ds;
        float min_range_sq, max_range_sq;
        size_t idx(size_t u, size_t v) const { return v * W + u; }
    };

    void assemble_image(Frame& frame);
    void estimate_normals(Frame& frame);

    ImageProjection proj_;
    Config cfg_;
};

} // namespace smip_uav

#endif