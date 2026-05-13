#include "surfel_map/frame_processor.hpp"
#include <iostream>

namespace smip_uav {

FrameProcessor::FrameProcessor(const Config& cfg) : config_(cfg) {}

std::vector<FrameSurfel> FrameProcessor::process(const Frame& cur_frame) {
    if (cur_frame.W == 0 || cur_frame.H == 0) return {};
    const size_t N = cur_frame.H * cur_frame.W;
    labels_.assign(N, -1);
    distances_.assign(N, std::numeric_limits<float>::max());

    init_seeds(cur_frame);

    // Run wavefront-based pixel clustering
    assign_pixels(cur_frame);
    update_seeds(cur_frame);

    // Extract resulting Surfel statistics from clusters
    std::vector<FrameSurfel> result = aggregate();
    return result;
}

void FrameProcessor::init_seeds(const Frame& f) {
    seeds_.clear();
    const int hw = static_cast<int>(config_.perturb_window);

    // Adaptive raster walk: step size S(z) = r_target / (pixel_pitch * z)
    // so each cluster covers ~r_target metres in 3D regardless of range.
    size_t v = config_.S_min / 2 + 1;
    while (v < f.H) {
        // Representative depth for vertical step: sample centre column of this row.
        const float d_row = f(f.W / 2, v).valid ? f(f.W / 2, v).depth : 0.0f;
        const size_t S_row = compute_S_local(d_row);

        size_t u = config_.S_min / 2 + 1;
        while (u < f.W) {
            // Local depth at candidate — fall back to nearby pixels if invalid.
            float d_here = f(u, v).valid ? f(u, v).depth : 0.0f;
            if (!std::isfinite(d_here) || d_here < 1e-3f) {
                for (int du = -2; du <= 2 && !(std::isfinite(d_here) && d_here > 1e-3f); ++du)
                    for (int dv2 = -2; dv2 <= 2 && !(std::isfinite(d_here) && d_here > 1e-3f); ++dv2) {
                        const int cu = static_cast<int>(u) + du;
                        const int cv = static_cast<int>(v) + dv2;
                        if (cu < 0 || cu >= static_cast<int>(f.W) || cv < 0 || cv >= static_cast<int>(f.H)) continue;
                        if (f(cu, cv).valid) d_here = f(cu, cv).depth;
                    }
            }
            const size_t S_local = compute_S_local(d_here);
            const float inv_S_local_sq = 1.0f / (static_cast<float>(S_local) * static_cast<float>(S_local));

            // Perturb to the pixel with the lowest depth gradient in the window.
            const int hw_eff = std::min(hw, static_cast<int>(S_local / 2 + 1));
            float best_grad = std::numeric_limits<float>::max();
            int best_u = -1, best_v = -1;
            for (int dv = -hw_eff; dv <= hw_eff; ++dv) {
                for (int du = -hw_eff; du <= hw_eff; ++du) {
                    const int cu = static_cast<int>(u) + du;
                    const int cv = static_cast<int>(v) + dv;
                    if (cu < 0 || cu >= static_cast<int>(f.W) || cv < 0 || cv >= static_cast<int>(f.H)) continue;
                    if (!f(cu, cv).valid) continue;
                    const float g = depth_gradient(f, cu, cv);
                    if (g < best_grad) { best_grad = g; best_u = cu; best_v = cv; }
                }
            }

            if (best_u >= 0) {
                const FramePixel& pxb = f(best_u, best_v);
                seeds_.push_back({
                    static_cast<float>(best_u),
                    static_cast<float>(best_v),
                    pxb.pos3d,
                    pxb.nrm3d,
                    pxb.depth,
                    inv_S_local_sq
                });
            }

            u += S_local;
        }
        v += S_row;
    }
}

void FrameProcessor::assign_pixels(const Frame& f) {
    bq_.clear();

    // seed the wavefront from all initial seeds
    for (size_t k = 0; k < seeds_.size(); ++k) {
        const Seed& seed = seeds_[k];
        const int su = static_cast<int>(std::round(seed.u));
        const int sv = static_cast<int>(std::round(seed.v));

        if (su < 0 || su >= static_cast<int>(f.W) || sv < 0 || sv >= static_cast<int>(f.H)) continue;

        const FramePixel& px = f(su, sv);
        if (!px.valid) continue;

        const size_t idx = f.idx(su, sv);
        const float d = distance(seed, su, sv, px);

        if (d < distances_[idx]) {
            distances_[idx] = d;
            labels_[idx] = static_cast<int32_t>(k);
            bq_.push(static_cast<uint32_t>(idx), d);
        }
    }

    // Expand wavefront - pop cycles until convergence
    uint32_t idx;
    while (bq_.pop(idx)) {
        const size_t u = idx % f.W;
        const size_t v = idx / f.W;
        const int32_t k = labels_[idx];
        if (k < 0) continue;

        struct NB {
            int du, dv;
        };
        static constexpr NB nbrs[4] = {{1,0}, {-1,0}, {0,1}, {0,-1}};

        for (const auto& nb : nbrs) {
            const int nu = static_cast<int>(u) + nb.du;
            const int nv = static_cast<int>(v) + nb.dv;

            if (nu < 0 || nu >= static_cast<int>(f.W) || nv < 0 || nv >= static_cast<int>(f.H)) continue;

            // edge check
            bool passable = false;
            if (nb.dv == 0) {
                // horizontal
                const size_t eu = std::min(u, static_cast<size_t>(nu));
                passable = f.edge_h[f.idx(eu, v)] != 0;
            }
            else {
                // vertical
                const size_t ev = std::min(v, static_cast<size_t>(nv));
                passable = f.edge_v[f.idx(u, ev)] != 0;
            }
            if (!passable) continue;

            const FramePixel& npx = f(nu, nv);
            if (!npx.valid) continue; // should not happen - depth map should catch this

            const float d_new = distance(seeds_[k], nu, nv, npx);
            if (d_new > config_.max_cluster_dist) continue;

            const size_t nidx = f.idx(nu,nv);
            if (d_new < distances_[nidx]) {
                distances_[nidx] = d_new;
                labels_[nidx] = k;
                bq_.push(static_cast<uint32_t>(nidx), d_new);
            }
        }
    }
}

void FrameProcessor::update_seeds(const Frame& f) {
    const size_t N_seeds = seeds_.size();
    seed_accums_.resize(N_seeds);
    for (auto& a : seed_accums_) a.reset();

    struct PlaneEstimate {
        Eigen::Vector3f centroid{Eigen::Vector3f::Zero()};
        Eigen::Vector3f normal{Eigen::Vector3f::Zero()};
        bool valid{false};
    };
    std::vector<PlaneEstimate> planes(N_seeds);

    // Single parallel region covers all three phases to avoid repeated fork/join
    #pragma omp parallel
    {
        // Thread-local accumulator — zero-initialized by SeedAccum default constructors
        std::vector<SeedAccum> local(N_seeds);

        // --- Pass 1: spatial Huber accumulation ---
        #pragma omp for schedule(static)
        for (size_t v = 0; v < f.H; ++v) {
            for (size_t u = 0; u < f.W; ++u) {
                const int32_t label = labels_[f.idx(u,v)];
                if (label < 0) continue;

                const FramePixel& px = f(u,v);
                if (!px.valid) continue;

                const Seed& seed = seeds_[label];
                const float r = (px.pos3d - seed.pos).norm();
                const float delta_h = config_.r_target;
                const float huber_scale = (r <= delta_h || r < 1e-6f) ? 1.0f : delta_h / r;
                const float w = px.weight * huber_scale;

                SeedAccum& a = local[label];
                a.sum_u     += w * static_cast<float>(u);
                a.sum_v     += w * static_cast<float>(v);
                a.sum_pos   += w * px.pos3d;
                a.sum_nrm   += w * px.nrm3d;
                a.sum_outer += w * px.pos3d * px.pos3d.transpose();
                a.sum_w     += w;
                a.count++;
            }
        }
        // Implicit barrier from omp for; reduce thread-locals sequentially into global
        #pragma omp critical
        for (size_t k = 0; k < N_seeds; ++k)
            seed_accums_[k].merge(local[k]);

        // All reductions must complete before eigensolves read seed_accums_
        #pragma omp barrier

        // --- Eigensolve phase: independent per seed, distributed across threads ---
        #pragma omp for schedule(dynamic, 4)
        for (size_t k = 0; k < N_seeds; ++k) {
            const auto& a = seed_accums_[k];
            if (a.count < config_.min_px || a.sum_w < 1e-8f) continue;

            const Eigen::Vector3f centroid = a.sum_pos / a.sum_w;
            const Eigen::Matrix3f cov = a.sum_outer / a.sum_w - centroid * centroid.transpose();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
            if (eig.info() != Eigen::Success) continue;

            Eigen::Vector3f normal = eig.eigenvectors().col(0);
            if (normal.dot(centroid) > 0.0f) normal = -normal;

            planes[k].centroid = centroid;
            planes[k].normal   = normal;
            planes[k].valid    = true;
        }
        // Implicit barrier from omp for

        // Reset seed_accums_ and local in parallel before pass 2
        #pragma omp for schedule(static) nowait
        for (size_t k = 0; k < N_seeds; ++k)
            seed_accums_[k].reset();

        for (auto& a : local) a.reset();

        // planes[] fully written and all resets done before pass 2 reads them
        #pragma omp barrier

        // --- Pass 2: point-to-plane Huber reaccumulation ---
        #pragma omp for schedule(static)
        for (size_t v = 0; v < f.H; ++v) {
            for (size_t u = 0; u < f.W; ++u) {
                const int32_t label = labels_[f.idx(u,v)];
                if (label < 0) continue;

                const FramePixel& px = f(u,v);
                if (!px.valid) continue;

                const Seed& seed = seeds_[label];
                float w = px.weight;

                const float r = (px.pos3d - seed.pos).norm();
                const float delta_h = config_.r_target;
                const float spatial_scale = (r <= delta_h || r < 1e-6f) ? 1.0f : delta_h / r;
                w *= spatial_scale;

                if (planes[label].valid) {
                    const Eigen::Vector3f& c = planes[label].centroid;
                    const Eigen::Vector3f& n = planes[label].normal;
                    const float residual = std::abs(n.dot(px.pos3d - c));
                    const float delta_p = 0.005f * px.depth * px.depth;
                    const float plane_scale = (residual <= delta_p || residual < 1e-6f)
                                               ? 1.0f : delta_p / residual;
                    w *= plane_scale;
                }

                SeedAccum& a = local[label];
                a.sum_u     += w * static_cast<float>(u);
                a.sum_v     += w * static_cast<float>(v);
                a.sum_pos   += w * px.pos3d;
                a.sum_nrm   += w * px.nrm3d;
                a.sum_outer += w * px.pos3d * px.pos3d.transpose();
                a.sum_w     += w;
                a.count++;
            }
        }
        // Implicit barrier from omp for; final reduction
        #pragma omp critical
        for (size_t k = 0; k < N_seeds; ++k)
            seed_accums_[k].merge(local[k]);

    } // end omp parallel — implicit barrier, all threads joined
}

float FrameProcessor::distance(const Seed& seed, size_t u, size_t v, const FramePixel& px) const {
    const float du = static_cast<float>(u) - seed.u;
    const float dv = static_cast<float>(v) - seed.v;
    const float d_img = (du * du + dv * dv) * seed.inv_S_local_sq;

    // c_spatial is now the fixed physical target radius, making d_spatial depth-independent.
    const float d_spatial = (px.pos3d - seed.pos).squaredNorm() / (config_.r_target * config_.r_target);

    const float n_dot = std::clamp(std::abs(px.nrm3d.dot(seed.nrm)), 0.0f, 1.0f);
    const float d_normal = (1.0f - n_dot) * (1.0f - n_dot);

    return d_img + config_.w_spatial * d_spatial + config_.w_normal * d_normal;
}

size_t FrameProcessor::compute_S_local(float depth) const {
    if (!std::isfinite(depth) || depth < 1e-3f) return config_.S_max;
    const float S_f = config_.r_target / (config_.pixel_pitch * depth);
    const auto S = static_cast<size_t>(std::round(S_f));
    return std::clamp(S, config_.S_min, config_.S_max);
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

std::vector<FrameSurfel> FrameProcessor::aggregate() const {
    const size_t N = seed_accums_.size();
    
    // preallocated (written by index)
    std::vector<FrameSurfel> slots(N);
    std::vector<uint8_t> filled(N,0);

    const float alpha = 0.005f; // ToF depth noise coefficient

    // Thread safe computations
    #pragma omp parallel for schedule(dynamic,4)
    for (size_t k = 0; k < N; ++k) {
        const auto& a = seed_accums_[k];
        if (a.count < config_.min_px || a.sum_w < 1e-8f) continue;
        
        const Eigen::Vector3f centroid = a.sum_pos / a.sum_w;
        if (!centroid.allFinite()) continue;

        // Weighted cluster covariance (local shape)
        const Eigen::Matrix3f C = a.sum_outer / a.sum_w - centroid * centroid.transpose();

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(C);
        if (eig.info() != Eigen::Success) continue;

        const Eigen::Vector3f evals = eig.eigenvalues().cwiseMax(0.0f);
        const Eigen::Matrix3f evecs = eig.eigenvectors();

        if (evals(1) < 1e-8f) continue;
        if (evals(0) / evals(1) > 0.2f) continue;

        Eigen::Vector3f normal = evecs.col(0);
        if (normal.dot(centroid) > 0.0f) normal = -normal;

        // Observation uncertainty covariance model:
        // First term: Surface estimation uncertainty: Neff increase -> more certain
        // Second term: range-direction uncertainty: Add a noise along the normal component of the cluster (increased with meaurement range)
        const float Neff = a.sum_w;
        const float r = centroid.norm();
        const float sigma_r = alpha * r * r;
        const Eigen::Matrix3f R = C / Neff + (sigma_r * sigma_r) * (normal * normal.transpose());

        // Fill surfel
        slots[k].sid = static_cast<uint32_t>(k); // index as id - no shared counter across threads
        slots[k].centroid = centroid;
        slots[k].normal = normal;
        slots[k].R = R;
        slots[k].eigenvalues = evals;
        slots[k].eigenvectors = evecs;
        slots[k].C_shape = C;
        slots[k].weight = Neff / static_cast<float>(a.count); // weight bounded [0,1]
        slots[k].view_cos_theta = -normal.dot(centroid.normalized()); // at surface-orthogonal inspection = 1
        filled[k] = 1;
    }

    // Push to output
    std::vector<FrameSurfel> surfels;
    surfels.reserve(N);
    for (size_t k = 0; k < N; ++k) {
        if (filled[k]) {
            surfels.push_back(slots[k]);
        }
    }
 
    return surfels;
}

} // smip_uav