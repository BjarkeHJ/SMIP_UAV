#ifndef FRAME_BUILDER_HPP_
#define FRAME_BUILDER_HPP_

#include "common/point_types.hpp"

namespace smip_uav {

struct GroundPlane {
    Eigen::Vector3f normal_z = Eigen::Vector3f::Zero();
    float offset_z = 0.0f;
};

class FrameBuilder {
public:
    struct Config {
        // Sensor specs
        size_t tof_res_x{180};
        size_t tof_res_y{240};
        float min_range{0.3f};
        float max_range{10.0f};
        float pixel_pitch{0.01071f}; //Radians per pixel -> DIRECTLY FROM SENSOR SPECS: 0.5(tan(hfov/2)/(resx/2) + tan(vfox/2)/resy/2)
        
        // Preprocessing
        bool transpose_input{true}; // True: Real Sensor data, False: Gz Sim data (message layout order is reversed also)
        bool enable_ground_filter{false};
        float ground_z_min{0.01f};
        int ds_factor{1};

        float edge_normal_th{0.52f}; // normal simi for edges ~30deg
        float edge_depth_min{0.02f}; // 2cm floor for close range
    };

    FrameBuilder() = default;
    explicit FrameBuilder(const Config& config);

    // Process the input pts (with optional ground plane filter)
    // return Frame datatype containing point, normal, weight information and can construct 2D images as well (depth, normal, weight)
    Frame process(const std::vector<PointXYZ>& pts, const int64_t timestamp, const GroundPlane* gnd = nullptr);

private:
    // projection geometry (immutable after construction
    struct Projection {
        size_t W, H, ds;
        float min_range_sq, max_range_sq;
        size_t idx(size_t u, size_t v) const { return v * W + u; }
    };

    void assemble_grid(const std::vector<PointXYZ>& pts, Frame& frame, const GroundPlane* gnd);
    void estimate_normals(Frame& frame);
    void compute_edges(Frame& frame);
    
    Config config_;
    Projection proj_;
};


} // smip uav

#endif