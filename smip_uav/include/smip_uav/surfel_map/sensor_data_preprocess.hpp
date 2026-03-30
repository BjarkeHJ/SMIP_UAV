#ifndef SENSOR_DATA_PREPROCESS_
#define SENSOR_DATA_PREPROCESS_

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "common/point_types.hpp"

namespace smip_uav {

struct GridCell {
    Eigen::Vector3f point = Eigen::Vector3f::Zero();
    float range_sq = std::numeric_limits<float>::infinity();
    bool valid = false;
};

struct GroundPlane {
    Eigen::Vector3f normal_z = Eigen::Vector3f::Zero();
    float offset_z = 0.0f;
};

class SensorDataPreprocess {
public:
    struct Config {
        // Sensor specs
        size_t tof_res_x{180};
        size_t tof_res_y{240};
        float min_range{0.3f};
        float max_range{10.0f};
        
        // Preprocessing
        bool transpose_input{false}; // True: Real Sensor data, False: Gz Sim data (message layout order is reversed also)
        bool enable_ground_filter{false};
        float ground_z_min{0.05f};
        int ds_factor{1};
        float jump_thresh{0.15};
    };

    SensorDataPreprocess() = default;
    explicit SensorDataPreprocess(const Config& config);

    // Process the input pts (with optional ground plane filter)
    // return Frame datatype containing point, normal, weight information and can construct 2D images as well (depth, normal, weight)
    Frame process(const std::vector<PointXYZ>& pts, const int64_t timestamp, const GroundPlane* gnd = nullptr) const;

private:
    // projection geometry (immutable after construction)
    struct Projection {
        size_t W, H, ds;
        float yaw_min, yaw_max;
        float pitch_min, pitch_max;
        float yaw_scale, pitch_scale;
        float min_range_sq, max_range_sq;
        size_t idx(size_t u, size_t v) const { return v * W + u; }
    };

    // scratch memory constructed each call
    struct Workspace {
        std::vector<GridCell> grid;
        std::vector<float> range_img;

        void resize(size_t n) {
            grid.resize(n);
            range_img.resize(n);
        }
    };

    void grid_downsample(const std::vector<PointXYZ>& pts, const GroundPlane* gnd, Workspace& ws) const;
    void build_range_image(Workspace& ws) const;
    void estimate_normals(const Workspace& ws, Frame& frame_out) const;

    Config config_;
    Projection proj_;
};


} // smip uav

#endif