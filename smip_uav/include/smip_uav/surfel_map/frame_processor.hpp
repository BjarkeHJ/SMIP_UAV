#ifndef FRAME_PROCESSOR_HPP_
#define FRAME_PROCESSOR_HPP_

#include "common/point_types.hpp"

namespace smip_uav {

class FrameProcessor {
public:
    struct Config {
        size_t max_slic_iter{3};
        size_t seed_spacing{10};
        size_t perturb_window{3};
        float convergence_px{0.5f};

        float w_spatial{1.0f};
        float w_normal{1.5f};

        size_t min_px{12};
        float min_total_w{5.0f};
    };
    
    FrameProcessor() = default;
    explicit FrameProcessor(const Config& cfg);
    
    std::vector<Surfel> process(const Frame& cur_frame);

private:
    struct Seed {
        // image seed for super pixel
        float u, v;
        Eigen::Vector3f pos;
        Eigen::Vector3f nrm;
        float depth;
    };

    struct SeedAccum {
        // Per-seed accumulative statistics
        float sum_u{0.0f}, sum_v{0.0f};
        Eigen::Vector3f sum_pos{Eigen::Vector3f::Zero()};
        Eigen::Vector3f sum_nrm{Eigen::Vector3f::Zero()};
        Eigen::Matrix3f sum_outer{Eigen::Matrix3f::Zero()};
        float sum_w{0.0f};
        uint32_t count{0};

        void reset() {
            sum_u = sum_v = sum_w = 0.0f;
            sum_pos.setZero();
            sum_nrm.setZero();
            sum_outer.setZero();
            count = 0;
        }
    };

    void init_seeds(const Frame& f);
    void assign_pixels(const Frame& f);
    float update_seeds(const Frame& f);
    std::vector<Surfel> aggregate() const;

    // Helpers
    float distance(const Seed& seed, size_t u, size_t v, const FramePixel& px) const;
    float depth_gradient(const Frame& f, size_t u, size_t v) const;

    // Buffers
    std::vector<Seed> seeds_;
    std::vector<SeedAccum> seed_accums_;
    std::vector<int32_t> labels_;
    std::vector<float> distances_;
    mutable uint32_t next_surfel_id_{0};

    // Temporal
    Frame prev_frame_;

    // State
    Config config_;
    
    // Derived Params
    float inv_S_sq_{0.0f};    
    float pixel_pitch_{0.0f};

};


} // smip_uav

#endif