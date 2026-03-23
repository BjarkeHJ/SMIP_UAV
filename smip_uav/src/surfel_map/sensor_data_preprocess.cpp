#include "surfel_map/sensor_data_preprocess.hpp"
#include <cmath>

namespace smip_uav {

SensorDataPreprocess::SensorDataPreprocess(const Config& config) : config_(config) {
    // derive projection geometry
    proj_.ds = std::max<size_t>(1, config.ds_factor);
    proj_.W  = (config.tof_res_x + proj_.ds - 1) / proj_.ds;
    proj_.W  = (config.tof_res_y + proj_.ds - 1) / proj_.ds;

    const float hfov = config_.hfov_deg * static_cast<float>(M_PI) / 180.0f;
    const float vfov = config_.vfov_deg * static_cast<float>(M_PI) / 180.0f;

    proj_.yaw_min = -hfov * 0.5f;
    proj_.yaw_max =  hfov * 0.5f;
    proj_.pitch_min = -vfov * 0.5f;
    proj_.pitch_max =  vfov * 0.5f;
    proj_.yaw_scale = (config_.tof_res_x  - 1) / hfov;
    proj_.pitch_scale = (config_.tof_res_y - 1) / vfov;
 
    proj_.min_range_sq = config_.min_range * config_.min_range;
    proj_.max_range_sq = config_.max_range * config_.max_range;
}

std::vector<PointNormal> SensorDataPreprocess::process(const std::vector<PointXYZ>& pts, const GroundPlane* gnd) const {
    if (pts.empty()) return {};

    const size_t n = proj_.W * proj_.H;
    Workspace ws;
    ws.resize(n);
 
    grid_downsample(pts, gnd, ws);
    build_range_image(ws);
    smooth_range_image(ws);
 
    std::vector<PointNormal> out;
    out.reserve(n);
    estimate_normals(ws, out);
    return out;
}

void SensorDataPreprocess::grid_downsample(const std::vector<PointXYZ>& pts, const GroundPlane* gnd, Workspace& ws) const {
    for (auto& cell : ws.grid) {
        cell.valid    = false;
        cell.range_sq = std::numeric_limits<float>::infinity();
    }
 
    const auto& P = proj_;
 
    for (const auto& p : pts) {
        if (!std::isfinite(p.px) || !std::isfinite(p.py) || !std::isfinite(p.pz))
            continue;
 
        const Eigen::Vector3f pv(p.px, p.py, p.pz);
        const float r_sq = pv.squaredNorm();
 
        if (r_sq < P.min_range_sq || r_sq > P.max_range_sq) continue;
 
        // optional ground filter
        if (config_.enable_ground_filter && gnd) {
            const float z_world = gnd->normal_z.dot(pv) + gnd->offset_z;
            if (z_world < config_.ground_z_min) continue;
        }
 
        const float yaw   = std::atan2(pv.y(), pv.x());
        const float pitch = std::atan2(pv.z(), std::hypot(pv.x(), pv.y()));
 
        if (yaw   < P.yaw_min   || yaw   > P.yaw_max)   continue;
        if (pitch < P.pitch_min || pitch > P.pitch_max) continue;
 
        const auto u = static_cast<size_t>((yaw   - P.yaw_min)   * P.yaw_scale   / P.ds + 0.5f);
        const auto v = static_cast<size_t>((pitch - P.pitch_min) * P.pitch_scale / P.ds + 0.5f);
 
        if (u >= P.W || v >= P.H) continue;
 
        GridCell& cell = ws.grid[P.idx(u, v)];
        if (r_sq < cell.range_sq) {
            cell.point    = pv;
            cell.range_sq = r_sq;
            cell.valid    = true;
        }
    }
}

void SensorDataPreprocess::build_range_image(Workspace& ws) const {
    const size_t n = proj_.W * proj_.H;
    for (size_t i = 0; i < n; ++i) {
        ws.range_img[i] = ws.grid[i].valid
            ? std::sqrt(ws.grid[i].range_sq)
            : std::numeric_limits<float>::infinity();
    }
}

void SensorDataPreprocess::smooth_range_image(Workspace& ws) const {
    if (config_.smooth_iters <= 0) return;
 
    ws.range_img_smooth = ws.range_img;
 
    const size_t R = std::max<size_t>(1, config_.normal_est_px_radius);
    const float inv_2s_sp = 1.0f / (2.0f * config_.spatial_sigma_px  * config_.spatial_sigma_px);
    const float inv_2s_dp = 1.0f / (2.0f * config_.depth_sigma_m     * config_.depth_sigma_m);
    const float max_jump  = config_.max_depth_jump_m;
    const auto& P = proj_;
 
    // precompute spatial kernel
    const size_t ksize = 2 * R + 1;
    std::vector<float> sp_kernel(ksize);
    for (size_t i = 0; i < ksize; ++i) {
        const float d = static_cast<float>(i) - static_cast<float>(R);
        sp_kernel[i] = std::exp(-d * d * inv_2s_sp);
    }
 
    // one bilateral pass (horizontal then vertical)
    auto bilateral_1d = [&](std::vector<float>& src, std::vector<float>& dst,
                            bool horizontal)
    {
        for (size_t v = 0; v < P.H; ++v) {
            for (size_t u = 0; u < P.W; ++u) {
                const size_t ic = P.idx(u, v);
                const float  rc = src[ic];
 
                if (!std::isfinite(rc)) { dst[ic] = rc; continue; }
 
                float wsum = 0.0f, vsum = 0.0f;
 
                // scan along the chosen axis
                const size_t axis_pos = horizontal ? u : v;
                const size_t axis_len = horizontal ? P.W : P.H;
                const size_t lo = (axis_pos > R) ? axis_pos - R : 0;
                const size_t hi = std::min(axis_len - 1, axis_pos + R);
 
                for (size_t k = lo; k <= hi; ++k) {
                    const size_t in = horizontal ? P.idx(k, v) : P.idx(u, k);
                    const float  rn = src[in];
                    if (!std::isfinite(rn)) continue;
 
                    const float dr = rn - rc;
                    if (std::fabs(dr) > max_jump) continue;
 
                    const float w = sp_kernel[k - axis_pos + R]
                                  * std::exp(-dr * dr * inv_2s_dp);
                    wsum += w;
                    vsum += w * rn;
                }
 
                dst[ic] = (wsum > 1e-6f) ? (vsum / wsum) : rc;
            }
        }
    };
 
    std::vector<float>* src = &ws.range_img_smooth;
    std::vector<float>* dst = &ws.temp;
 
    for (size_t iter = 0; iter < config_.smooth_iters; ++iter) {
        bilateral_1d(*src, *dst, /*horizontal=*/true);
        std::swap(src, dst);
        bilateral_1d(*src, *dst, /*horizontal=*/false);
        std::swap(src, dst);
    }
 
    if (src != &ws.range_img_smooth)
        ws.range_img_smooth = *src;
}

void SensorDataPreprocess::estimate_normals(const Workspace& ws, std::vector<PointNormal>& out) const {
    const auto& P = proj_;
    const float jump_thresh = config_.max_depth_jump_m;
 
    const std::vector<float>& range =
        (config_.smooth_iters > 0) ? ws.range_img_smooth : ws.range_img;
 
    // fetch a valid neighbour or nullopt
    auto neighbour = [&](size_t u, size_t v, int du, int dv) -> std::pair<bool, Eigen::Vector3f> {
        const size_t uu = u + du, vv = v + dv;
        if (uu >= P.W || vv >= P.H) return {false, {}};
 
        const size_t in = P.idx(uu, vv);
        if (!ws.grid[in].valid) return {false, {}};
 
        const float rn = range[in];
        if (!std::isfinite(rn) || std::fabs(rn - range[P.idx(u, v)]) > jump_thresh)
            return {false, {}};
 
        return {true, ws.grid[in].point};
    };
 
    // central / forward / backward difference
    auto finite_diff = [](bool okA, const Eigen::Vector3f& A, bool okB, const Eigen::Vector3f& B, const Eigen::Vector3f& C) -> std::pair<bool, Eigen::Vector3f> {
        if (okA && okB) return {true, B - A};
        if (okB) return {true, B - C};
        if (okA) return {true, C - A};
        return {false, {}};
    };
 
    const float alpha = 1.0f / (proj_.max_range_sq);
 
    for (size_t v = 0; v < P.H; ++v) {
        for (size_t u = 0; u < P.W; ++u) {
            const size_t ic = P.idx(u, v);
            if (!ws.grid[ic].valid) continue;
 
            const Eigen::Vector3f& Pc = ws.grid[ic].point;
 
            auto [okL, Pl] = neighbour(u, v, -1,  0);
            auto [okR, Pr] = neighbour(u, v, +1,  0);
            auto [okU, Pu] = neighbour(u, v,  0, -1);
            auto [okD, Pd] = neighbour(u, v,  0, +1);
 
            auto [hasU, tu] = finite_diff(okL, Pl, okR, Pr, Pc);
            auto [hasV, tv] = finite_diff(okU, Pu, okD, Pd, Pc);
            if (!hasU || !hasV) continue;
 
            Eigen::Vector3f normal = tu.cross(tv);
            const float nn = normal.norm();
            if (nn < 1e-6f) continue;
            normal /= nn;
 
            if (normal.dot(Pc) > 0.0f)
                normal = -normal;
 
            // weighting
            const float r   = Pc.norm();
            const float w_range     = 1.0f / (1.0f + alpha * r * r);
 
            const float cos_inc     = std::abs(normal.dot(-Pc.normalized()));
            const float w_incidence = std::pow(cos_inc, 0.5f);
 
            const float un = tu.norm(), vn = tv.norm();
            const float q_anis      = 2.0f * std::min(un, vn) / (un + vn + 1e-6f);
            const float sin_theta   = nn / (un * vn);
            const float w_quality   = sin_theta * q_anis;
 
            PointNormal pn;
            pn.px = Pc.x();
            pn.py = Pc.y();
            pn.pz = Pc.z();
            pn.nx = normal.x();
            pn.ny = normal.y();
            pn.nz = normal.z();
            pn.w = w_range * w_incidence * w_quality;
            out.push_back(pn);
        }
    }
}

} // smip_uav