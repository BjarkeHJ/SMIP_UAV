#include "surfel_map/sensor_data_preprocess.hpp"
#include <cmath>

namespace smip_uav {

SensorDataPreprocess::SensorDataPreprocess(const Config& config) : config_(config) {
    // derive projection geometry
    proj_.ds = std::max<size_t>(1, config.ds_factor);
    proj_.W  = (config.tof_res_x + proj_.ds - 1) / proj_.ds;
    proj_.H  = (config.tof_res_y + proj_.ds - 1) / proj_.ds;
    proj_.min_range_sq = config_.min_range * config_.min_range;
    proj_.max_range_sq = config_.max_range * config_.max_range;
}

Frame SensorDataPreprocess::process(const std::vector<PointXYZ>& pts, const int64_t timestamp, const GroundPlane* gnd) const {
    if (pts.empty()) return Frame(proj_.W, proj_.H, 0);

    const size_t n = proj_.W * proj_.H;
    Workspace ws;
    ws.resize(n);
 
    grid_downsample(pts, gnd, ws);
    build_range_image(ws);
 
    Frame frame_out(proj_.W, proj_.H, timestamp);
    estimate_normals(ws, frame_out);
    return frame_out;
}

void SensorDataPreprocess::grid_downsample(const std::vector<PointXYZ>& pts, const GroundPlane* gnd, Workspace& ws) const {
    for (auto& cell : ws.grid) {
        cell.valid    = false;
        cell.range_sq = std::numeric_limits<float>::infinity();
    }
 
    const auto& P = proj_;
 
    // for (const auto& p : pts) {
    for (size_t i = 0; i < pts.size(); ++i) {
        const auto& p = pts[i];
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
 
        // reject points behind the camera
        if (pv.x() <= 0.0f) continue;
        
        // pixel coords from array index
        size_t u, v;
        if (config_.transpose_input) {
            u = (i / config_.tof_res_y) / P.ds;
            v = (i % config_.tof_res_y) / P.ds;
        } else {
            u = P.W - 1 - (i % config_.tof_res_x) / P.ds;
            v = P.H - 1 - (i / config_.tof_res_x) / P.ds;
        }
                
        if (u >= P.W || v >= P.H) continue;
 
        GridCell& cell = ws.grid[P.idx(u, v)];
        if (r_sq < cell.range_sq) {
            cell.point = pv;
            cell.range_sq = r_sq;
            cell.valid = true;
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

void SensorDataPreprocess::estimate_normals(const Workspace& ws, Frame& frame_out) const {
    const auto& P = proj_; 
    const std::vector<float>& range = ws.range_img;
 
    // fetch a valid neighbour or nullopt
    auto neighbour = [&](size_t u, size_t v, int du, int dv) -> std::pair<bool, Eigen::Vector3f> {
        const size_t uu = u + du, vv = v + dv;
        if (uu >= P.W || vv >= P.H) return {false, {}};
 
        const size_t in = P.idx(uu, vv);
        if (!ws.grid[in].valid) return {false, {}};
 
        const float rn = range[in];
        if (!std::isfinite(rn) || std::fabs(rn - range[P.idx(u, v)]) > config_.jump_thresh)
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
            const float r = Pc.norm();
            const float w_range = 1.0f / (1.0f + alpha * r * r);
 
            const float cos_inc = std::abs(normal.dot(-Pc.normalized()));
            const float w_incidence = std::pow(cos_inc, 0.5f);
 
            const float un = tu.norm();
            const float vn = tv.norm();
            const float q_anis = 2.0f * std::min(un, vn) / (un + vn + 1e-6f);
            const float sin_theta = nn / (un * vn);
            const float w_quality = sin_theta * q_anis;
 
            PointNormal pn;
            pn.px = Pc.x();
            pn.py = Pc.y();
            pn.pz = Pc.z();
            pn.nx = normal.x();
            pn.ny = normal.y();
            pn.nz = normal.z();
            pn.w = w_range * w_incidence * w_quality;

            frame_out(u, v) = pn;
        }
    }
}

} // smip_uav