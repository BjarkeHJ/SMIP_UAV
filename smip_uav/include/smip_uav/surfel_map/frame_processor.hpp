#ifndef FRAME_PROCESSOR_HPP_
#define FRAME_PROCESSOR_HPP_

#include "common/point_types.hpp"
#include <random>

namespace smip_uav {

class FrameProcessor {
public:
    struct Config {
        // Adaptive seed spacing: seeds are placed so each surfel covers ~r_target metres in 3D.
        // S(z) = r_target / (pixel_pitch * z), clamped to [S_min, S_max].
        float  r_target{0.3f};    // [m]  target physical surfel radius
        size_t S_min{4};          // [px] minimum pixel seed spacing (dense far-range limit)
        size_t S_max{10};         // [px] maximum pixel seed spacing (sparse near-range / no-depth fallback)

        size_t perturb_window{1};
        size_t min_px{12};

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
        float u, v;
        Eigen::Vector3f pos;
        Eigen::Vector3f nrm;
        float depth;
        float inv_S_local_sq; // 1 / S_local^2, where S_local = r_target / (pixel_pitch * depth)
    };

    struct SeedAccum {
        // Per-seed accumulative statistics
        Eigen::Vector3f sum_pos{Eigen::Vector3f::Zero()};
        Eigen::Vector3f sum_nrm{Eigen::Vector3f::Zero()};
        Eigen::Matrix3f sum_outer{Eigen::Matrix3f::Zero()};
        float sum_w{0.0f};
        uint32_t count{0};

        void reset() {
            sum_w = 0.0f;
            sum_pos.setZero();
            sum_nrm.setZero();
            sum_outer.setZero();
            count = 0;
        }

        void merge(const SeedAccum& o) {
            sum_pos   += o.sum_pos;
            sum_nrm   += o.sum_nrm;
            sum_outer += o.sum_outer;
            sum_w     += o.sum_w;
            count     += o.count;
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
    size_t compute_S_local(float depth) const;

    // Buffers
    BucketQueue bq_;

    std::vector<Seed> seeds_;
    std::vector<SeedAccum> seed_accums_;
    std::vector<int32_t> labels_;
    std::vector<float> distances_;
    // State
    Config config_;
    std::mt19937 rng_{std::random_device{}()};


};


} // smip_uav

#endif