#ifndef SMIP_SURFEL_EXTRACTOR_HPP_
#define SMIP_SURFEL_EXTRACTOR_HPP_

#include <array>
#include <random>
#include <Eigen/Eigenvalues>

#include "core/frame.hpp"
#include "core/surfel.hpp"

namespace smip_uav {

class SurfelExtractor {
public:
    struct Config {
        // Adaptive seed spacing: seeds are placed so each surfel covers ~r_target metres in 3D.
        // S(z) = r_target / (pixel_pitch * z), clamped to [S_min, S_max].
        float  r_target{0.2f};
        size_t S_min{4};
        size_t S_max{10};

        size_t perturb_window{1};
        size_t min_px{12};

        float w_spatial{1.0f};
        float w_normal{1.0f};
        float max_cluster_dist{1.0f};
        
        float pixel_pitch{0.01071f};

        float tof_alpha{0.005f};
        float planarity_sigma_mult{3.0f};
        float confidence_sigma_ref_sq{1e-4f};
    };

    SurfelExtractor() : SurfelExtractor(Config{}) {}
    explicit SurfelExtractor(const Config& cfg);

    void extract(Frame& frame);

    const std::vector<int32_t>& labels() const { return labels_; }

private:
    struct Seed {
        float u, v;
        Eigen::Vector3f pos;
        Eigen::Vector3f nrm;
        float depth;
        float inv_S_local_sq;
    };

    struct PlaneEstimate {
        Eigen::Vector3f centroid{Eigen::Vector3f::Zero()};
        Eigen::Vector3f normal{Eigen::Vector3f::Zero()};
        Eigen::Matrix3f shape_cov{Eigen::Matrix3f::Zero()};  // 3×3 scatter in tangent plane
        Eigen::Vector3f evals{Eigen::Vector3f::Zero()};  // eigenvalues (λ₀ ≤ λ₁ ≤ λ₂)
        float sum_w{0.0f};
        uint32_t count{0};
        bool valid{false};
    };

    struct SeedAccum {
        Eigen::Vector3f sum_pos{Eigen::Vector3f::Zero()};
        Eigen::Vector3f sum_nrm{Eigen::Vector3f::Zero()};
        // Upper-triangle of symmetric w*p*pᵀ sum: [00,01,02,11,12,22]
        float outer[6]{};
        float sum_w{0.0f};
        uint32_t count{0};

        void reset() {
            sum_pos.setZero(); sum_nrm.setZero();
            for (auto& x : outer) x = 0.0f;
            sum_w = 0.0f; count = 0;
        }

        void merge(const SeedAccum& o) {
            sum_pos += o.sum_pos;
            sum_nrm += o.sum_nrm;
            for (int i = 0; i < 6; ++i) outer[i] += o.outer[i];
            sum_w += o.sum_w;
            count += o.count;
        }

        Eigen::Matrix3f sum_outer_matrix() const {
            Eigen::Matrix3f m;
            m << outer[0], outer[1], outer[2],
                 outer[1], outer[3], outer[4],
                 outer[2], outer[4], outer[5];
            return m;
        }
    };

    struct BucketQueue {
        static constexpr size_t NUM_BUCKETS  = 256;
        static constexpr float  BUCKET_WIDTH = 0.05f;
        std::array<std::vector<uint32_t>, NUM_BUCKETS> buckets;
        size_t current_bucket{0};

        void push(uint32_t idx, float dist) {
            const size_t b = std::min<size_t>(
                static_cast<size_t>(dist / BUCKET_WIDTH), NUM_BUCKETS - 1);
            buckets[b].push_back(idx);
            if (b < current_bucket) current_bucket = b;
        }
        bool pop(uint32_t& idx) {
            while (current_bucket < NUM_BUCKETS) {
                auto& bkt = buckets[current_bucket];
                if (!bkt.empty()) { idx = bkt.back(); bkt.pop_back(); return true; }
                ++current_bucket;
            }
            return false;
        }
        void clear() { for (auto& b : buckets) b.clear(); current_bucket = 0; }
    };

    void init_seeds(const Frame& f);
    void assign_pixels(const Frame& f);
    void update_seeds(const Frame& f);
    std::vector<Surfel> aggregate() const;

    float  distance(const Seed& seed, size_t u, size_t v, const Eigen::Vector3f& pos, const Eigen::Vector3f& nrm) const;
    float  depth_gradient(size_t u, size_t v) const;
    size_t compute_S_local(float depth) const;

    // Per-frame state (reset at start of each extract() call)
    size_t W_{0}, H_{0};
    std::vector<float> depths_;  // sqrt(ranges[j]), 0 for invalid pixels

    // Persistent scratch buffers
    BucketQueue bq_;
    std::vector<Seed> seeds_;
    std::vector<SeedAccum> seed_accums_;
    std::vector<PlaneEstimate> planes_;
    std::vector<std::vector<SeedAccum>> tlocal_;  // thread-local accumulators, reused each frame
    std::vector<int32_t> labels_;
    std::vector<float> distances_;

    Config cfg_;
    std::mt19937 rng_{std::random_device{}()};
};

} // namespace smip_uav

#endif
