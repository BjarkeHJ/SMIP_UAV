#ifndef SENSOR_DATA_PREPROCESS_
#define SENSOR_DATA_PREPROCESS_

#include "common/point_types.hpp"

namespace smip_uav {

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
    };

    SensorDataPreprocess();



private:

    Config config_;
};


} // smip uav

#endif