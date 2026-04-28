#include "surfel_map/frame_builder.hpp"

namespace smip_uav {

FrameBuilder::FrameBuilder(const Config& config) : config_(config) {
    // derive projection geometry
    proj_.ds = std::max<size_t>(1, config.ds_factor);
    proj_.W  = (config.tof_res_x + proj_.ds - 1) / proj_.ds;
    proj_.H  = (config.tof_res_y + proj_.ds - 1) / proj_.ds;
    proj_.min_range_sq = config_.min_range * config_.min_range;
    proj_.max_range_sq = config_.max_range * config_.max_range;
}

Frame FrameBuilder::process(const std::vector<PointXYZ>& pts, const int64_t timestamp, const GroundPlane* gnd) {
    if (pts.empty()) return Frame(proj_.W, proj_.H, 0);

    // Construct current frame from sensor geometry
    Frame frame(proj_.W, proj_.H, timestamp);

    assemble_grid(pts, frame, gnd);
    estimate_normals(frame);
    compute_edges(frame);

    return frame;
}

void FrameBuilder::assemble_grid(const std::vector<PointXYZ>& pts, Frame& frame, const GroundPlane* gnd) {
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
 
        // reject points behind the camera (z axis aligned with view)
        if (pv.z() <= 0.0f) continue;
        
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
 
        // Keep closest
        FramePixel& px = frame(u, v);
        if (!px.valid || r_sq < px.depth * px.depth) {
            px.pos3d = pv;
            px.depth = std::sqrt(r_sq);
            px.valid = true;
        }
    }
}

void FrameBuilder::estimate_normals(Frame& frame) {
    const auto& P = proj_; 
    const float pp = config_.pixel_pitch;

    // Side buffer for avoiding races while setting px validity state flag in parallel
    std::vector<uint8_t> kill(P.W * P.H, 0);    

    // fetch a valid neighbour or nullopt
    auto fetch_nb = [&](float cur_depth, size_t nb_u, size_t nb_v) -> std::pair<bool, Eigen::Vector3f> {
        if (nb_u >= P.W || nb_v >= P.H) return {false, {}};
        
        // Neighbor
        FramePixel& px_nb = frame(nb_u,nb_v);
        if (!px_nb.valid) return {false, {}};
        
        const float r_avg = 0.5f * (cur_depth + px_nb.depth); // average depth
        const float tau = 2.0f * r_avg * pp + config_.edge_depth_min; // multiplier of 2 to increase tolerance
        if (std::fabs(px_nb.depth - cur_depth) > tau) return {false, {}};
 
        return {true, px_nb.pos3d};
    };
 
    // central / forward / backward difference
    auto finite_diff = [](bool okA, const Eigen::Vector3f& A, bool okB, const Eigen::Vector3f& B, const Eigen::Vector3f& C) -> std::pair<bool, Eigen::Vector3f> {
        if (okA && okB) return {true, B - A};
        if (okB) return {true, B - C};
        if (okA) return {true, C - A};
        return {false, {}};
    };
 
    const float alpha = 1.0f / (config_.max_range);
    
    #pragma omp parallel for schedule(static)
    for (size_t v = 0; v < P.H; ++v) {
        for (size_t u = 0; u < P.W; ++u) {
            FramePixel& px = frame(u,v);
            if (!px.valid) continue;
 
            const Eigen::Vector3f& Pc = px.pos3d;
 
            auto [okL, Pl] = fetch_nb(px.depth, u-1, v);
            auto [okR, Pr] = fetch_nb(px.depth, u+1, v);
            auto [okU, Pu] = fetch_nb(px.depth, u, v-1);
            auto [okD, Pd] = fetch_nb(px.depth, u, v+1);
 
            auto [hasU, tu] = finite_diff(okL, Pl, okR, Pr, Pc);
            auto [hasV, tv] = finite_diff(okU, Pu, okD, Pd, Pc);
            if (!hasU || !hasV) {
                kill[frame.idx(u,v)] = 1; // set invalid in buffer
                // px.valid = false;
                continue;
            }
            Eigen::Vector3f normal = tu.cross(tv);
            const float nn = normal.norm();
            if (nn < 1e-6f) {
                kill[frame.idx(u,v)] = 1; // set invalid in buffer
                // px.valid = false;
                continue;
            }
            normal /= nn;
 
            if (normal.dot(Pc) > 0.0f)
                normal = -normal;
 
            // weighting
            const float r = px.depth;
            const Eigen::Vector3f Pc_unit = Pc * (1.0f / r);   // one multiply, no sqrt
            const float w_incidence = std::abs(normal.dot(Pc_unit)); // cos of incidence angle
            const float w_range = 1.0f / (1.0f + alpha * r * r);
 
            const float un = tu.norm();
            const float vn = tv.norm();
            const float q_anis = 2.0f * std::min(un, vn) / (un + vn + 1e-6f);
            const float sin_theta = nn / (un * vn);
            const float w_quality = sin_theta * q_anis;
 
            px.nrm3d = normal;
            px.weight = w_range * w_incidence * w_quality;
        }
    }

    // Apply the validity state updates
    for (size_t i = 0; i < kill.size(); ++i) {
        if (kill[i]) frame.pixels[i].valid = false;
    }
}

void FrameBuilder::compute_edges(Frame& frame) {
    const float cos_thresh = std::cos(config_.edge_normal_th);
    const float pp = config_.pixel_pitch;
    
    #pragma omp parallel for schedule(static)
    for (size_t v = 0; v < frame.H; ++v) {
        for (size_t u = 0; u < frame.W; ++u) {
            const FramePixel& px = frame(u, v);
            if (!px.valid) continue;

            // Horizontal edge: (u,v) -> (u+1,v)
            if (u + 1 < frame.W) {
                const FramePixel& pr = frame(u+1,v);
                if (pr.valid) {
                    const float r_avg = 0.5f * (px.depth + pr.depth);
                    const float tau = r_avg * pp + config_.edge_depth_min;
                    const float dd = std::abs(px.depth - pr.depth);
                    const float nn = px.nrm3d.dot(pr.nrm3d);
                    if (dd <= tau && nn >= cos_thresh) {
                        frame.edge_h[frame.idx(u,v)] = 1; // passable
                    }
                }
            }

            // Vertical edge: (u,v) -> (u,v+1)
            if (v + 1 < frame.H) {
                const FramePixel& pd = frame(u,v+1);
                if (pd.valid) {
                    const float r_avg = 0.5f * (px.depth + pd.depth);
                    const float tau = r_avg * pp + config_.edge_depth_min;
                    const float dd = std::abs(px.depth - pd.depth);
                    const float nn = px.nrm3d.dot(pd.nrm3d);
                    if (dd <= tau && nn >= cos_thresh) {
                        frame.edge_v[frame.idx(u,v)] = 1; // passable
                    }
                }
            }
        }
    }
}

} // smip_uav