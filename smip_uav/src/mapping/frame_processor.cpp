#include "mapping/frame_processor.hpp"

namespace smip_uav {

FrameProcessor::FrameProcessor(const Config& config) : cfg_(config) {
    proj_.ds = std::max<size_t>(1, cfg_.ds_factor);
    proj_.W = (cfg_.tof_res_x + proj_.ds - 1) / proj_.ds;
    proj_.H = (cfg_.tof_res_y + proj_.ds - 1) / proj_.ds;
    proj_.min_range_sq = cfg_.min_range * cfg_.min_range;
    proj_.max_range_sq = cfg_.max_range * cfg_.max_range;
}

void FrameProcessor::process(Frame& frame) {
    // Fill Frame Fields
    assemble_image(frame);
    estimate_normals(frame);
}

void FrameProcessor::assemble_image(Frame& frame) {

    for (size_t i = 0; i < frame.size(); ++i) {
        const auto& pn = frame.pixels.pointnormals[i];
        if (!pn.p.allFinite()) continue;

        const float r_sq = pn.p.squaredNorm();
        if (r_sq < proj_.min_range_sq || r_sq > proj_.max_range_sq) continue;

        size_t u, v;
        u = (i % cfg_.tof_res_x) / proj_.ds;
        v = (i / cfg_.tof_res_x) / proj_.ds;

        // TEST ALL THIS WITH DATA TO ENSURE ALIGNMENT WITH SENSOR ORDERING
        // u = proj_.H - 1 - (i % cfg_.tof_res_y) / proj_.ds;
        // v = proj_.W - 1 - (i / cfg_.tof_res_y) / proj_.ds;

        if (u >= proj_.W || v >= proj_.H) continue;
        
        const size_t j = proj_.idx(u,v);
        if (r_sq < frame.pixels.ranges[j]) {
            frame.pixels.validities[j] = 1;
            frame.pixels.ranges[j] = r_sq;
            frame.pixels.pointnormals[j].p = pn.p;
        }
    }
}

void FrameProcessor::estimate_normals(Frame& frame) {
    const float pp    = cfg_.pixel_pitch;
    const float alpha = 1.0f / cfg_.max_range;

    std::vector<uint8_t> kill(proj_.W * proj_.H, 0);

    auto fetch_nb = [&](float cur_depth, size_t nb_u, size_t nb_v) -> std::pair<bool, Eigen::Vector3f> {
        if (nb_u >= proj_.W || nb_v >= proj_.H) return {false, {}};
        const size_t nb_j = proj_.idx(nb_u, nb_v);
        if (!frame.pixels.validities[nb_j]) return {false, {}};
        const float nb_depth = std::sqrt(frame.pixels.ranges[nb_j]);
        const float tau = 0.5f * (cur_depth + nb_depth) * pp;
        if (std::fabs(nb_depth - cur_depth) > tau) return {false, {}};
        return {true, frame.pixels.pointnormals[nb_j].p};
    };

    auto finite_diff = [](bool okA, const Eigen::Vector3f& A, bool okB, const Eigen::Vector3f& B, const Eigen::Vector3f& C) -> std::pair<bool, Eigen::Vector3f> {
        if (okA && okB) return {true, B - A};
        if (okB)        return {true, B - C};
        if (okA)        return {true, C - A};
        return {false, {}};
    };

    #pragma omp parallel for schedule(static)
    for (size_t v = 0; v < proj_.H; ++v) {
        for (size_t u = 0; u < proj_.W; ++u) {
            const size_t j = proj_.idx(u, v);
            if (!frame.pixels.validities[j]) continue;

            const float depth = std::sqrt(frame.pixels.ranges[j]);
            Eigen::Vector3f& Pc = frame.pixels.pointnormals[j].p;

            auto [okL, Pl] = fetch_nb(depth, u-1, v);
            auto [okR, Pr] = fetch_nb(depth, u+1, v);
            auto [okU, Pu] = fetch_nb(depth, u, v-1);
            auto [okD, Pd] = fetch_nb(depth, u, v+1);

            auto [hasU, tu] = finite_diff(okL, Pl, okR, Pr, Pc);
            auto [hasV, tv] = finite_diff(okU, Pu, okD, Pd, Pc);
            if (!hasU || !hasV) { kill[j] = 1; continue; }

            Eigen::Vector3f normal = tu.cross(tv);
            const float nn = normal.norm();
            if (nn < 1e-6f) { kill[j] = 1; continue; }
            normal /= nn;

            if (normal.dot(Pc) > 0.0f) normal = -normal;

            const Eigen::Vector3f Pc_unit = Pc * (1.0f / depth);
            const float w_incidence = std::abs(normal.dot(Pc_unit));
            const float w_range = 1.0f / (1.0f + alpha * depth);
            const float un = tu.norm();
            const float vn = tv.norm();
            const float q_anis = 2.0f * std::min(un, vn) / (un + vn + 1e-6f);
            const float sin_theta = nn / (un * vn);
            const float w_quality = sin_theta * q_anis;

            frame.pixels.pointnormals[j].n = normal;
            frame.pixels.weights[j] = w_range * w_incidence * w_quality;
        }
    }

    for (size_t i = 0; i < kill.size(); ++i) {
        if (kill[i]) frame.pixels.validities[i] = 0;
    }
}





} // namespace smip_uav