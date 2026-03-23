#ifndef SENSOR_DATA_PREPROCESS_
#define SENSOR_DATA_PREPROCESS_

#include <vector>
#include <Eigen/Core>
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
        bool enable_ground_filter{false};
        float ground_z_min{0.2f};

        // Sensor specs
        size_t tof_res_x{240};
        size_t tof_res_y{180};
        float hfov_deg{106.0f};
        float vfov_deg{86.0f};
        float min_range{0.1f};
        float max_range{10.0f};

        int ds_factor{1};
        int normal_est_px_radius{3};

        size_t smooth_iters{3};
        float depth_sigma_m{0.05f};
        float spatial_sigma_px{1.0f};
        float max_depth_jump_m{0.1f};
    };

    explicit SensorDataPreprocess(const Config& config = {});

    // Process the input pts (with optional ground plane filter)
    // return vector of points with corresponding normals and measurement weight
    std::vector<PointNormal> process(const std::vector<PointXYZ>& pts, const GroundPlane* gnd = nullptr) const;

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
        std::vector<float> range_img_smooth;
        std::vector<float> temp;

        void resize(size_t n) {
            grid.resize(n);
            range_img.resize(n);
            range_img_smooth.resize(n);
            temp.resize(n);
        }
    };

    void grid_downsample(const std::vector<PointXYZ>& pts, const GroundPlane* gnd, Workspace& ws) const;
    void build_range_image(Workspace& ws) const;
    void smooth_range_image(Workspace& ws) const;
    void estimate_normals(const Workspace& ws, std::vector<PointNormal>& pns_out) const;

    Config config_;
    Projection proj_;
};


} // smip uav

#endif