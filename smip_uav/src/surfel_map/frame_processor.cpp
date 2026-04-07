#include "surfel_map/frame_processor.hpp"
#include <iostream>

namespace smip_uav {

FrameProcessor::FrameProcessor(const Config& cfg) : config_(cfg) {
    const float S = static_cast<float>(config_.seed_spacing);
    inv_S_sq_ = 1.0f / (S * S);
    pixel_pitch_ = 0.5 * (std::tan(86.0f / 180.0f * M_PI / 180.0f) + std::tan(106.0f / 240.0f * M_PI / 180)); // DIRECTLY FROM SENSOR SPECS 0.5(hfov/px_res_x + vfov/px_res_y)
}

std::vector<Surfel> FrameProcessor::process(const Frame& cur_frame) {
    if (cur_frame.W == 0 || cur_frame.H == 0) return {};
    const size_t N = cur_frame.H * cur_frame.W;
    labels_.assign(N, -1);

    init_seeds(cur_frame);
    std::cout << "Number of seeds: " << seeds_.size() << std::endl;

    // Run SLIC-Based pixel clustering 
    for (size_t iter = 0; iter < config_.max_slic_iter; ++iter) {
        assign_pixels(cur_frame);
        const float max_shift = update_seeds(cur_frame);
        if (max_shift < config_.convergence_px) {
            std::cout << "Finished early, iter: " << iter + 1 << std::endl;
            break;
        }
    }

    // Extract resulting Surfel statistics from clusters
    std::vector<Surfel> result = aggregate();
    std::cout << "Surfels: " << result.size() << std::endl;
    return result;
}

void FrameProcessor::init_seeds(const Frame& f) {
    seeds_.clear();
    const size_t S = config_.seed_spacing;
    const size_t u0 = S / 2;
    const size_t v0 = S / 2;
    const int hw = static_cast<int>(config_.perturb_window);

    for (size_t v = v0; v < f.H; v += S) {
        for (size_t u = u0; u < f.W; u += S) {
            // assign seed to the smallest depth gradient in the neighborhood
            float best_grad = std::numeric_limits<float>::max();
            int best_u = -1;
            int best_v = -1;

            for (int dv = -hw; dv <= hw; ++dv) {
                for (int du = -hw; du <= hw; ++du) {
                    const int cu = static_cast<int>(u) + du;
                    const int cv = static_cast<int>(v) + dv;
                    if (cu < 0 || cu >= static_cast<int>(f.W) || cv < 0 || cv >= static_cast<int>(f.H)) continue;

                    const FramePixel& px = f(cu, cv);
                    if (!px.valid) continue;

                    const float g = depth_gradient(f, cu, cv);
                    if (g < best_grad) {
                        best_grad = g;
                        best_u = cu;
                        best_v = cv;
                    }
                }
            }

            if (best_u < 0 || best_v < 0) continue; // no valid pixel in nbh

            const FramePixel& pxb = f(best_u, best_v);
            seeds_.push_back({
                static_cast<float>(best_u),
                static_cast<float>(best_v),
                pxb.pos3d,
                pxb.nrm3d,
                pxb.depth
            });
        }
    }
}

void FrameProcessor::assign_pixels(const Frame& f) {
    const int search = 2 * static_cast<int>(config_.seed_spacing);
    distances_.assign(f.H*f.W, std::numeric_limits<float>::max());
    
    for (size_t k = 0; k < seeds_.size(); ++k) {
        const Seed& seed = seeds_[k];
        const int su = static_cast<int>(std::round(seed.u));    
        const int sv = static_cast<int>(std::round(seed.v));

        const int u_min = std::max(0, su - search);
        const int u_max = std::min(static_cast<int>(f.W) - 1, su + search);
        const int v_min = std::max(0, sv - search);
        const int v_max = std::min(static_cast<int>(f.H) - 1, sv + search);

        for (int v = v_min; v <= v_max; ++v) {
            for (int u = u_min; u <= u_max; ++u) {
                const FramePixel& px = f(u,v);
                if (!px.valid) continue;

                const float d = distance(seed, u, v, px);
                const size_t idx = f.idx(u, v);

                if (d < distances_[idx]) {
                    distances_[idx] = d;
                    labels_[idx] = static_cast<int32_t>(k);
                }
            }
        }
    }
}

float FrameProcessor::update_seeds(const Frame& f) {
    seed_accums_.resize(seeds_.size());
    for (auto& a : seed_accums_) a.reset();

    // accumulated over labelled points
    for (size_t v = 0; v < f.H; ++v) {
        for (size_t u = 0; u < f.W; ++u) {
            const int32_t label = labels_[f.idx(u,v)];
            if (label < 0) continue; // unlabled

            const FramePixel& px = f(u,v);
            if (!px.valid) continue;

            SeedAccum& a = seed_accums_[label];
            const float w = px.weight;
            
            a.sum_u += w*u;
            a.sum_v += w*v;
            a.sum_pos += w*px.pos3d;
            a.sum_nrm += w*px.nrm3d;
            a.sum_outer += w * px.pos3d * px.pos3d.transpose();
            a.sum_w += w;
            a.count++;
        }
    }

    // track max shift in centroid (convergence)
    float max_shift_sq = 0.0f;
    for (size_t k = 0; k < seeds_.size(); ++k) {
        const auto& a = seed_accums_[k];
        if (a.count == 0 || a.sum_w < 1e-12) continue;
 
        auto& seed = seeds_[k];
 
        const float new_u = static_cast<float>(a.sum_u / a.sum_w);
        const float new_v = static_cast<float>(a.sum_v / a.sum_w);
 
        const float du = new_u - seed.u;
        const float dv = new_v - seed.v;
        max_shift_sq = std::max(max_shift_sq, du * du + dv * dv);
 
        seed.u = new_u;
        seed.v = new_v;
        seed.pos = (a.sum_pos / a.sum_w).cast<float>();
 
        Eigen::Vector3f nrm_avg = (a.sum_nrm / a.sum_w).cast<float>();
        const float nrm_len = nrm_avg.norm();
        seed.nrm = (nrm_len > 1e-6f) ? (nrm_avg / nrm_len) : seed.nrm;
        seed.depth = seed.pos.norm();
    }
 
    return std::sqrt(max_shift_sq);
}

float FrameProcessor::distance(const Seed& seed, size_t u, size_t v, const FramePixel& px) const {
    const float du = static_cast<float>(u) - seed.u;
    const float dv = static_cast<float>(v) - seed.v;
    const float d_img = (du * du + dv * dv) * inv_S_sq_;

    const float c_spatial = seed.depth * pixel_pitch_ * config_.seed_spacing + 0.001f; // 0.001f from min range (10cm)
    const float d_spatial = (px.pos3d - seed.pos).squaredNorm() / (c_spatial * c_spatial);

    const float n_dot = std::clamp(std::abs(px.nrm3d.dot(seed.nrm)), 0.0f, 1.0f);
    const float d_normal = (1.0f - n_dot) * (1.0f - n_dot);

    return d_img + config_.w_spatial * d_spatial + config_.w_normal * d_normal;
}

float FrameProcessor::depth_gradient(const Frame& f, size_t u, size_t v) const {
    constexpr float INDETERMINATE = std::numeric_limits<float>::max();

    bool has_gx = false;
    bool has_gy = false;
    float gx = 0.0f;
    float gy = 0.0f;

    if (u > 0 && u+1 < f.W) {
        const FramePixel& l = f(u-1, v);
        const FramePixel& r = f(u+1, v);
        if (l.valid && r.valid) {
            gx = r.depth - l.depth;
            has_gx = true;
        }
    }
    if (v > 0 && v+1 < f.H) {
        const FramePixel& t = f(u, v+1);
        const FramePixel& b = f(u, v-1);
        if (t.valid && b.valid) {
            gy = b.depth - t.depth;
            has_gy = true;
        }
    }

    if (!has_gx && !has_gy) return INDETERMINATE;
    
    return gx * gx + gy * gy;
}

std::vector<Surfel> FrameProcessor::aggregate() const {
    std::vector<Surfel> surfels;
    surfels.reserve(seed_accums_.size());
 
    for (size_t k = 0; k < seed_accums_.size(); ++k) {
        const auto& a = seed_accums_[k];
 
        if (a.count < config_.min_px) continue;
        if (a.sum_w < config_.min_total_w) continue;
 
        Surfel sf;
        sf.sid = next_surfel_id_++;
        sf.W  = a.sum_w;
        sf.S1 = a.sum_pos;
        sf.S2 = a.sum_outer;    // raw Σ wᵢ pᵢ pᵢᵀ

        surfels.push_back(sf);
    }
 
    return surfels;
}

} // smip_uav