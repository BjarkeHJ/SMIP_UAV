#ifndef FRAME_PROCESSOR_HPP_
#define FRAME_PROCESSOR_HPP_

#include "common/point_types.hpp"

namespace smip_uav {

class FrameProcessor {
public:
    struct Config {
        size_t seed_spacing{4};
        size_t perturb_window{1};
        size_t min_px{8};

        float w_spatial{1.0f};
        float w_normal{1.0f};
        float max_cluster_dist{1.0f};
        float pixel_pitch{0.01071f}; //DIRECTLY FROM SENSOR SPECS: 0.5(tan(hfov/2)/(resx/2) + tan(vfox/2)/resy/2)
    };
    
    FrameProcessor() = default;
    explicit FrameProcessor(const Config& cfg);

    std::vector<FrameSurfel> process(const Frame& cur_frame);

    // Per-pixel surfel label after the last process() call (-1 = unassigned)
    const std::vector<int32_t>& labels() const { return labels_; }

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

    struct BucketQueue {
        static constexpr size_t NUM_BUCKETS = 256;
        static constexpr float BUCKET_WIDTH = 0.05f;
        std::array<std::vector<uint32_t>, NUM_BUCKETS> buckets;
        size_t current_bucket = 0;

        void push(uint32_t pixel_idx, float dist) {
            size_t b = std::min<size_t>(
                static_cast<size_t>(dist / BUCKET_WIDTH),
                NUM_BUCKETS - 1
            );
            buckets[b].push_back(pixel_idx);
        }

        bool pop(uint32_t& pixel_idx) {
            while (current_bucket < NUM_BUCKETS) {
                auto& bkt = buckets[current_bucket];
                if (!bkt.empty()) {
                    pixel_idx = bkt.back();
                    bkt.pop_back();
                    return true;
                }
                ++current_bucket;
            }
            return false;
        }

        void clear() {
            for (auto& b : buckets) b.clear();
            current_bucket = 0;
        }
    };

    void init_seeds(const Frame& f);
    void assign_pixels(const Frame& f);
    void update_seeds(const Frame& f);
    std::vector<FrameSurfel> aggregate() const;

    // Helpers
    float distance(const Seed& seed, size_t u, size_t v, const FramePixel& px) const;
    float depth_gradient(const Frame& f, size_t u, size_t v) const;

    // Buffers
    BucketQueue bq_;

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

};


} // smip_uav

#endif