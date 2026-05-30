#include "active_map/surfel_extractor.hpp"
#include <omp.h>

namespace smip_uav {

SurfelExtractor::SurfelExtractor(const Config& cfg) : cfg_(cfg) {
    for (auto& b : bq_.buckets)
        b.reserve(43200 / BucketQueue::NUM_BUCKETS);
}

void SurfelExtractor::extract(Frame& frame) {
    W_ = frame.meta.width;
    H_ = frame.meta.height;
    if (W_ == 0 || H_ == 0) return;

    const size_t N = W_ * H_;
    labels_.assign(N, -1);
    distances_.assign(N, std::numeric_limits<float>::max());

    // Precompute depths once — avoids repeated sqrt throughout all passes.
    // depths_[j] == 0 signals invalid (validities[j] == 0).
    depths_.resize(N);
    for (size_t j = 0; j < N; ++j)
        depths_[j] = frame.pixels.validities[j] ? std::sqrt(frame.pixels.ranges[j]) : 0.0f;

    init_seeds(frame);
    assign_pixels(frame);
    update_seeds(frame);

    // Finalize surfel in frame
    frame.surfels = aggregate();
}

// ---------------------------------------------------------------------------
// init_seeds
// ---------------------------------------------------------------------------
void SurfelExtractor::init_seeds(const Frame& f) {
    seeds_.clear();
    const int hw = static_cast<int>(cfg_.perturb_window);

    std::uniform_int_distribution<size_t> offset_dist(0, cfg_.S_min - 1);
    const size_t u_offset = offset_dist(rng_);
    const size_t v_offset = offset_dist(rng_);

    size_t v = v_offset + cfg_.S_min / 2 + 1;
    while (v < H_) {
        const size_t cj  = v * W_ + W_ / 2;
        const float d_row = depths_[cj] > 1e-3f ? depths_[cj] : 0.0f;
        const size_t S_row = compute_S_local(d_row);

        size_t u = u_offset + cfg_.S_min / 2 + 1;
        while (u < W_) {
            float d_here = depths_[v * W_ + u];

            // Fall back to nearby pixels if invalid.
            if (!std::isfinite(d_here) || d_here < 1e-3f) {
                for (int du = -2; du <= 2 && !(std::isfinite(d_here) && d_here > 1e-3f); ++du)
                    for (int dv2 = -2; dv2 <= 2 && !(std::isfinite(d_here) && d_here > 1e-3f); ++dv2) {
                        const int cu = static_cast<int>(u) + du;
                        const int cv = static_cast<int>(v) + dv2;
                        if (cu < 0 || cu >= static_cast<int>(W_) ||
                            cv < 0 || cv >= static_cast<int>(H_)) continue;
                        const float d = depths_[cv * W_ + cu];
                        if (d > 1e-3f) d_here = d;
                    }
            }

            const size_t S_local = compute_S_local(d_here);
            const float inv_S_local_sq =
                1.0f / (static_cast<float>(S_local) * static_cast<float>(S_local));

            const int jitter_half = static_cast<int>(S_local) / 3;
            const int ju = std::uniform_int_distribution<int>(-jitter_half, jitter_half)(rng_);
            const int jv = std::uniform_int_distribution<int>(-jitter_half, jitter_half)(rng_);
            const int u_cand = std::clamp(static_cast<int>(u) + ju, 0, static_cast<int>(W_) - 1);
            const int v_cand = std::clamp(static_cast<int>(v) + jv, 0, static_cast<int>(H_) - 1);

            const int hw_eff = std::min(hw, static_cast<int>(S_local / 2 + 1));
            float best_grad = std::numeric_limits<float>::max();
            int best_u = -1, best_v = -1;
            for (int dv = -hw_eff; dv <= hw_eff; ++dv) {
                for (int du = -hw_eff; du <= hw_eff; ++du) {
                    const int cu = u_cand + du;
                    const int cv = v_cand + dv;
                    if (cu < 0 || cu >= static_cast<int>(W_) ||
                        cv < 0 || cv >= static_cast<int>(H_)) continue;
                    if (depths_[cv * W_ + cu] < 1e-3f) continue;
                    const float g = depth_gradient(cu, cv);
                    if (g < best_grad) { best_grad = g; best_u = cu; best_v = cv; }
                }
            }

            if (best_u >= 0) {
                const size_t bj = best_v * W_ + best_u;
                seeds_.push_back({
                    static_cast<float>(best_u),
                    static_cast<float>(best_v),
                    f.pixels.pointnormals[bj].p,
                    f.pixels.pointnormals[bj].n,
                    depths_[bj],
                    inv_S_local_sq
                });
            }

            u += S_local;
        }
        v += S_row;
    }
}

// ---------------------------------------------------------------------------
// assign_pixels  (wavefront BFS)
// Edge passability is computed inline via depth-discontinuity check —
// the new Frame has no pre-stored edge_h/edge_v arrays.
// ---------------------------------------------------------------------------
void SurfelExtractor::assign_pixels(const Frame& f) {
    bq_.clear();

    for (size_t k = 0; k < seeds_.size(); ++k) {
        const Seed& seed = seeds_[k];
        const int su = static_cast<int>(std::round(seed.u));
        const int sv = static_cast<int>(std::round(seed.v));
        if (su < 0 || su >= static_cast<int>(W_) ||
            sv < 0 || sv >= static_cast<int>(H_)) continue;

        const size_t idx = sv * W_ + su;
        if (!f.pixels.validities[idx]) continue;

        const float d = distance(seed, su, sv,
                                 f.pixels.pointnormals[idx].p,
                                 f.pixels.pointnormals[idx].n);
        if (d < distances_[idx]) {
            distances_[idx] = d;
            labels_[idx]    = static_cast<int32_t>(k);
            bq_.push(static_cast<uint32_t>(idx), d);
        }
    }

    uint32_t idx;
    while (bq_.pop(idx)) {
        const size_t u = idx % W_;
        const size_t v = idx / W_;
        const int32_t k = labels_[idx];
        if (k < 0) continue;

        struct NB { int du, dv; };
        static constexpr NB nbrs[4] = {{1,0},{-1,0},{0,1},{0,-1}};

        for (const auto& nb : nbrs) {
            const int nu = static_cast<int>(u) + nb.du;
            const int nv = static_cast<int>(v) + nb.dv;
            if (nu < 0 || nu >= static_cast<int>(W_) ||
                nv < 0 || nv >= static_cast<int>(H_)) continue;

            const size_t nidx = nv * W_ + nu;

            // Depth-discontinuity edge check (replaces edge_h / edge_v lookup)
            const float d0 = depths_[idx];
            const float d1 = depths_[nidx];
            if (d0 < 1e-3f || d1 < 1e-3f) continue;
            const float tau = 0.5f * (d0 + d1) * cfg_.pixel_pitch;
            if (std::fabs(d1 - d0) > tau) continue;

            if (!f.pixels.validities[nidx]) continue;

            const float d_new = distance(seeds_[k], nu, nv,
                                         f.pixels.pointnormals[nidx].p,
                                         f.pixels.pointnormals[nidx].n);
            if (d_new > cfg_.max_cluster_dist) continue;

            if (d_new < distances_[nidx]) {
                distances_[nidx] = d_new;
                labels_[nidx]    = k;
                bq_.push(static_cast<uint32_t>(nidx), d_new);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// update_seeds  (two-pass Huber accumulation + eigensolve)
// ---------------------------------------------------------------------------
void SurfelExtractor::update_seeds(const Frame& f) {
    const size_t N_seeds = seeds_.size();
    seed_accums_.resize(N_seeds);
    planes_.resize(N_seeds);

    for (auto& a : seed_accums_) a.reset();
    for (auto& p : planes_) p = PlaneEstimate{};
    
    const float delta_h_sq = cfg_.r_target * cfg_.r_target;

    // -----------------------------------------------------------------------
    // Pass 1: spatial Huber → accumulate into thread-local buffers → reduce
    // -----------------------------------------------------------------------
    {
        const int nthreads = omp_get_max_threads();
        std::vector<std::vector<SeedAccum>> tlocal(nthreads,
                                                    std::vector<SeedAccum>(N_seeds));
 
        #pragma omp parallel
        {
            const int tid = omp_get_thread_num();
            auto& local = tlocal[tid];
 
            #pragma omp for schedule(static)
            for (size_t v = 0; v < H_; ++v) {
                for (size_t u = 0; u < W_; ++u) {
                    const size_t j     = v * W_ + u;
                    const int32_t label = labels_[j];
                    if (label < 0 || !f.pixels.validities[j]) continue;
 
                    const Eigen::Vector3f& pos = f.pixels.pointnormals[j].p;
                    const Eigen::Vector3f& nrm = f.pixels.pointnormals[j].n;
                    const Seed& seed = seeds_[label];
 
                    const float r_sq = (pos - seed.pos).squaredNorm();
                    const float huber_scale = (r_sq <= delta_h_sq || r_sq < 1e-6f)
                                              ? 1.0f : cfg_.r_target / std::sqrt(r_sq);
                    const float w = f.pixels.weights[j] * huber_scale;
 
                    auto& a     = local[label];
                    a.sum_pos   += w * pos;
                    a.sum_nrm   += w * nrm;
                    a.sum_outer += w * pos * pos.transpose();
                    a.sum_w     += w;
                    a.count++;
                }
            }
        } // end parallel (implicit barrier)
 
        // Serial reduction across threads
        std::vector<SeedAccum> pass1(N_seeds);
        for (int t = 0; t < nthreads; ++t)
            for (size_t k = 0; k < N_seeds; ++k)
                pass1[k].merge(tlocal[t][k]);
 
        // Eigensolve: derive planes_ from pass-1 statistics
        #pragma omp parallel for schedule(dynamic, 4)
        for (size_t k = 0; k < N_seeds; ++k) {
            const auto& a = pass1[k];
            if (a.count < cfg_.min_px || a.sum_w < 1e-8f) continue;
 
            const Eigen::Vector3f centroid = a.sum_pos / a.sum_w;
            const Eigen::Matrix3f cov =
                a.sum_outer / a.sum_w - centroid * centroid.transpose();
 
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
            if (eig.info() != Eigen::Success) continue;
 
            const Eigen::Vector3f evals = eig.eigenvalues().cwiseMax(0.0f);
            const Eigen::Matrix3f evecs = eig.eigenvectors();
 
            Eigen::Vector3f normal = evecs.col(0);
            if (normal.dot(centroid) > 0.0f) normal = -normal;
 
            // Shape covariance: project scatter onto tangent plane (zero out
            // normal direction so the matrix reflects spatial extent only).
            // C_shape = C - λ₀ * n*nᵀ  (removes the "thickness" component)
            const Eigen::Matrix3f shape_cov =
                cov - evals(0) * (normal * normal.transpose());
 
            planes_[k].centroid  = centroid;
            planes_[k].normal    = normal;
            planes_[k].shape_cov = shape_cov;
            planes_[k].evals     = evals;
            planes_[k].sum_w     = a.sum_w;
            planes_[k].count     = a.count;
            planes_[k].valid     = true;
        }
    } // pass1 local storage freed here
 
    // -----------------------------------------------------------------------
    // Pass 2: point-to-plane Huber reaccumulation → seed_accums_
    // -----------------------------------------------------------------------
    {
        const int nthreads = omp_get_max_threads();
        std::vector<std::vector<SeedAccum>> tlocal(nthreads,
                                                    std::vector<SeedAccum>(N_seeds));
 
        #pragma omp parallel
        {
            const int tid = omp_get_thread_num();
            auto& local = tlocal[tid];
 
            #pragma omp for schedule(static)
            for (size_t v = 0; v < H_; ++v) {
                for (size_t u = 0; u < W_; ++u) {
                    const size_t j      = v * W_ + u;
                    const int32_t label = labels_[j];
                    if (label < 0 || !f.pixels.validities[j]) continue;
 
                    const Eigen::Vector3f& pos = f.pixels.pointnormals[j].p;
                    const Eigen::Vector3f& nrm = f.pixels.pointnormals[j].n;
                    const Seed& seed = seeds_[label];
 
                    float w = f.pixels.weights[j];
 
                    // Spatial Huber
                    const float r = (pos - seed.pos).norm();
                    const float spatial_scale = (r <= cfg_.r_target || r < 1e-6f)
                                                ? 1.0f : cfg_.r_target / r;
                    w *= spatial_scale;
 
                    // Point-to-plane Huber
                    if (planes_[label].valid) {
                        const Eigen::Vector3f& c = planes_[label].centroid;
                        const Eigen::Vector3f& n = planes_[label].normal;
                        const float residual  = std::abs(n.dot(pos - c));
                        // delta_p proportional to range (ranges = depth²)
                        const float delta_p   = 0.005f * f.pixels.ranges[j];
                        const float plane_scale = (residual <= delta_p || residual < 1e-6f)
                                                  ? 1.0f : delta_p / residual;
                        w *= plane_scale;
                    }
 
                    auto& a     = local[label];
                    a.sum_pos   += w * pos;
                    a.sum_nrm   += w * nrm;
                    a.sum_outer += w * pos * pos.transpose();
                    a.sum_w     += w;
                    a.count++;
                }
            }
        } // end parallel
 
        for (int t = 0; t < nthreads; ++t)
            for (size_t k = 0; k < N_seeds; ++k)
                seed_accums_[k].merge(tlocal[t][k]);
    }
}

// ---------------------------------------------------------------------------
// aggregate  — maps accumulated statistics → Surfel
// ---------------------------------------------------------------------------
std::vector<Surfel> SurfelExtractor::aggregate() const {
    const size_t N = seed_accums_.size();
    std::vector<Surfel>  slots(N);
    std::vector<uint8_t> filled(N, 0);
 
    #pragma omp parallel for schedule(dynamic, 4)
    for (size_t k = 0; k < N; ++k) {
        const auto& a  = seed_accums_[k];
        const auto& pl = planes_[k];
 
        if (!pl.valid) continue;
        if (a.count < cfg_.min_px || a.sum_w < 1e-8f) continue;
 
        // --- Planarity gate (depth-aware) ---
        // Expected normal-direction variance due to range noise alone.
        const float r = pl.centroid.norm();
        const float sigma_r   = cfg_.tof_alpha * r * r;
        const float lambda_noise = sigma_r * sigma_r;
        if (pl.evals(0) > cfg_.planarity_sigma_mult * lambda_noise) continue;
        if (pl.evals(1) < 1e-8f) continue;  // degenerate cluster
 
        // --- Weighted mean position/normal from pass-2 ---
        const Eigen::Vector3f centroid = a.sum_pos / a.sum_w;
        if (!centroid.allFinite()) continue;
 
        // Use pass-1 normal (more robust: computed on larger weight set).
        // Recomputing from pass-2 covariance would require another eigensolve.
        Eigen::Vector3f normal = pl.normal;
        // Flip guard (sensor is unlikely to be inside the surface).
        if (normal.dot(centroid) > 0.0f) normal = -normal;
 
        // Average normal direction from pixel normals (weighted).
        // For consistency with downstream fusion, normalise the sum_nrm.
        Eigen::Vector3f avg_nrm = a.sum_nrm;
        const float nrm_len = avg_nrm.norm();
        if (nrm_len > 1e-6f) avg_nrm /= nrm_len;
        // Blend: prefer geometry-derived normal but allow pixel normals to
        // disambiguate in ambiguous (near-90°) cases.
        normal = (0.7f * normal + 0.3f * avg_nrm).normalized();
        if (normal.dot(centroid) > 0.0f) normal = -normal;
 
        // --- Measurement covariance R ---
        // C_pass2: scatter from pass-2 (tighter than pass-1, plane-weighted)
        const Eigen::Matrix3f C_pass2 = a.sum_outer / a.sum_w - centroid * centroid.transpose();
 
        // Centroid estimation uncertainty: divide by count (sample covariance
        // of the mean), NOT by sum_w which conflates weights and sample count.
        const float inv_count = 1.0f / static_cast<float>(a.count);
        const Eigen::Matrix3f R_estimation = C_pass2 * inv_count;
 
        // Range noise: σ_r = tof_alpha * r²
        const float sigma_r_sq = sigma_r * sigma_r;
        const Eigen::Matrix3f R_range = sigma_r_sq * (normal * normal.transpose());
 
        // Lateral noise floor: σ_tan = pixel_pitch * r / 2
        const float sigma_tan = cfg_.pixel_pitch * r * 0.5f;
        const float sigma_tan_sq = sigma_tan * sigma_tan;
        const Eigen::Matrix3f I3 = Eigen::Matrix3f::Identity();
        const Eigen::Matrix3f R_lateral = sigma_tan_sq * (I3 - normal * normal.transpose());
 
        const Eigen::Matrix3f R = R_estimation + R_range + R_lateral;
 
        // --- Inlier ratio (mean Huber weight in pass-2) ---
        const float inlier_ratio = std::min(1.0f, a.sum_w / static_cast<float>(a.count));
 
        // --- Confidence: combines inlier quality with measurement precision ---
        const float tr_R = R.trace();
        const float confidence = inlier_ratio * std::exp(-tr_R / cfg_.confidence_sigma_ref_sq);
 
   
        // --- Fill surfel ---
        Surfel& s = slots[k];
        s.position = centroid;
        s.normal = normal;
        s.covariance = R;
        s.shape = pl.shape_cov;   // full 3×3 tangent-plane extent
        s.inlier_ratio = inlier_ratio;
        s.confidence = confidence;
        s.obs_count = a.count;
 
        filled[k] = 1;
    }
 
    std::vector<Surfel> out;
    out.reserve(N);
    for (size_t k = 0; k < N; ++k)
        if (filled[k]) out.push_back(slots[k]);
    return out;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
float SurfelExtractor::distance(const Seed& seed, size_t u, size_t v,
                                const Eigen::Vector3f& pos,
                                const Eigen::Vector3f& nrm) const {
    const float du = static_cast<float>(u) - seed.u;
    const float dv = static_cast<float>(v) - seed.v;
    const float d_img     = (du * du + dv * dv) * seed.inv_S_local_sq;
    const float d_spatial = (pos - seed.pos).squaredNorm() /
                            (cfg_.r_target * cfg_.r_target);
    const float n_dot     = std::clamp(std::abs(nrm.dot(seed.nrm)), 0.0f, 1.0f);
    const float d_normal  = (1.0f - n_dot) * (1.0f - n_dot);
    return d_img + cfg_.w_spatial * d_spatial + cfg_.w_normal * d_normal;
}

size_t SurfelExtractor::compute_S_local(float depth) const {
    if (!std::isfinite(depth) || depth < 1e-3f) return cfg_.S_max;
    const float S_f = cfg_.r_target / (cfg_.pixel_pitch * depth);
    return std::clamp(static_cast<size_t>(std::round(S_f)), cfg_.S_min, cfg_.S_max);
}

float SurfelExtractor::depth_gradient(size_t u, size_t v) const {
    constexpr float INDETERMINATE = std::numeric_limits<float>::max();
    bool has_gx = false, has_gy = false;
    float gx = 0.0f, gy = 0.0f;

    if (u > 0 && u + 1 < W_) {
        const float dl = depths_[v * W_ + (u - 1)];
        const float dr = depths_[v * W_ + (u + 1)];
        if (dl > 1e-3f && dr > 1e-3f) { gx = dr - dl; has_gx = true; }
    }
    if (v > 0 && v + 1 < H_) {
        const float dt = depths_[(v + 1) * W_ + u];
        const float db = depths_[(v - 1) * W_ + u];
        if (dt > 1e-3f && db > 1e-3f) { gy = db - dt; has_gy = true; }
    }
    if (!has_gx && !has_gy) return INDETERMINATE;
    return gx * gx + gy * gy;
}

} // namespace smip_uav
