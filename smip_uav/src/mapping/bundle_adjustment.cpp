#include "mapping/bundle_adjustment.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

namespace smip_uav {

namespace {
// SO(3) helpers
inline Eigen::Matrix3d so3_hat(const Eigen::Vector3d& w) {
    Eigen::Matrix3d W;
    W << 0.0, -w.z(), w.y(),
        w.z(), 0.0, -w.x(),
        -w.y(), w.x(), 0.0;
    return W;
}
inline Eigen::Matrix3d so3_exp(const Eigen::Vector3d& w) {
    const double th = w.norm();
    if (th < 1e-12) return Eigen::Matrix3d::Identity() + so3_hat(w);
    const Eigen::Vector3d a = w / th;
    return Eigen::AngleAxisd(th, a).toRotationMatrix();
}
inline Eigen::Vector3d so3_log(const Eigen::Matrix3d& R) {
    const Eigen::AngleAxisd aa(R);
    return aa.angle() * aa.axis();
}
inline Eigen::Isometry3d perturb_pose(const Eigen::Isometry3d& T, int dim, double h) {
    Eigen::Isometry3d out = T;
    if (dim < 3) {
        Eigen::Vector3d w = Eigen::Vector3d::Zero();
        w[dim] = h;
        out.linear() = so3_exp(w) * T.rotation();
    }
    else {
        out.translation()[dim - 3] += h;
    }
    return out;
}
inline void apply_delta(Eigen::Isometry3d& T, const Eigen::Matrix<double, 6, 1>& d) {
    T.linear() = so3_exp(d.head<3>()) * T.rotation();
    T.translation() += d.tail<3>();
}

// World-frame voxel-hash 
struct VoxKey {
    int32_t x,y,z;
    bool operator==(const VoxKey& k) const { return x == k.x && y == k.y && z == k.z; }
};
struct VoxKeyHash {
    size_t operator()(const VoxKey& k) const noexcept {
        size_t h = 2166136261u;
        auto mix = [&](uint32_t v) { h ^= static_cast<size_t>(v); h *= 16777619u; };
        mix(static_cast<uint32_t>(k.x));
        mix(static_cast<uint32_t>(k.y));
        mix(static_cast<uint32_t>(k.z));
        return h;
    }
};
inline VoxKey to_key(const Eigen::Vector3d& p, double inv_vox) {
    return { static_cast<int32_t>(std::floor(p.x() * inv_vox)),
             static_cast<int32_t>(std::floor(p.y() * inv_vox)),
             static_cast<int32_t>(std::floor(p.z() * inv_vox))};
}

// odometry chain residual
inline Eigen::Matrix<double, 6, 1> chain_residual(
    const Eigen::Isometry3d& Ti, const Eigen::Isometry3d& Tj,
    const Eigen::Isometry3d& Z, double inv_sr, double inv_st) {
    const Eigen::Isometry3d E = Z.inverse() * (Ti.inverse() * Tj);
    Eigen::Matrix<double, 6, 1> r;
    r.head<3>() = so3_log(E.rotation()) * inv_sr;
    r.tail<3>() = E.translation() * inv_st;
    return r;
}

inline double now_ms() {
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

} // anon ns


// INGEST: Surfel -> exact point-cluster moments, submap-local, cached for-ever
void BundleAdjustment::ingest_submap(SubmapId id, const FrozenSubmap& fs) {
    if (has(id)) return;

    CachedSubmap cs;
    cs.id = id;
    cs.T_origin = fs.T_submap_world_origin.cast<double>();
    cs.surfels.reserve(fs.surfels.size());

    for (const Surfel& s : fs.surfels) {
        double w = static_cast<double>(s.weight);
        if (w <= 0.0 && s.confidence > 0.0f) {
            const double c = std::min(static_cast<double>(s.confidence), 0.999);
            w = -static_cast<double>(cfg_.fallback_confidence_ref) * std::log(1.0 - c);
        }

        if (w < 1e-6) continue;
        if (s.normal.squaredNorm() < 0.5f) continue;

        CachedSurfel c;
        c.mu = s.position.cast<double>();
        c.n = s.normal.cast<double>();
        c.W = w;
        c.S1 = w * c.mu;
        c.S2 = w * (s.shape.cast<double>() + c.mu * c.mu.transpose());
        cs.surfels.push_back(std::move(c));
    }

    id_to_cache_[id] = static_cast<uint32_t>(cache_.size());
    cache_.push_back(std::move(cs));
}

BundleAdjustment::TransformedContribution BundleAdjustment::transform_contribution(const Contribution& c, const Eigen::Isometry3d& T) {
    // moment transform under SE(3): closed form maeks evaluation O(#submaps)
    const Eigen::Matrix3d R = T.rotation();
    const Eigen::Vector3d t = T.translation();
    TransformedContribution out;
    out.W = c.W;
    const Eigen::Vector3d RS1 = R * c.S1;
    out.S1 = RS1 + c.W * t;
    out.S2 = R * c.S2 * R.transpose() + RS1 * t.transpose() + t * RS1.transpose() + c.W * (t * t.transpose());
    return out;
}

double BundleAdjustment::cluster_cost(double Wt, const Eigen::Vector3d& S1t, const Eigen::Matrix3d& S2t) const {
    if (Wt < 1e-9) return 0.0;
    const Eigen::Vector3d mu = S1t / Wt;
    const Eigen::Matrix3d A = S2t / Wt - mu * mu.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
    es.computeDirect(A, Eigen::EigenvaluesOnly);
    const double lmin = std::max(0.0, es.eigenvalues()(0));
    const double s = static_cast<double>(cfg_.ba_sigma_m);
    return lmin / (s * s);
}

void BundleAdjustment::associate(const std::vector<Eigen::Isometry3d>& poses, Stats& st) {
    clusters_.clear();
    st.clusters = 0;
    st.dropped_few_submaps = 0;
    st.dropped_planarity = 0;
    st.dropped_weight = 0;

    struct Ref {
        uint32_t sm;
        uint32_t si;
        Eigen::Vector3f mu_w;
        Eigen::Vector3f n_w;
        float w;
    };

    std::vector<Ref> refs;
    size_t total = 0;
    for (const auto& cs : cache_) total += cs.surfels.size();
    refs.reserve(total);

    const double inv_vox = 1.0 / static_cast<double>(cfg_.assoc_voxel_size);
    std::unordered_map<VoxKey, std::vector<uint32_t>, VoxKeyHash> grid;
    grid.reserve(total / 2 + 1);

    for (uint32_t sm = 0; sm < cache_.size(); ++sm) {
        const Eigen::Isometry3d& T = poses[sm];
        const Eigen::Matrix3d& R = T.rotation();
        for (uint32_t si = 0; si < cache_[sm].surfels.size(); ++si) {
            const CachedSurfel& s = cache_[sm].surfels[si];
            const Eigen::Vector3d mu_w = T * s.mu;
            Ref r;
            r.sm = sm;
            r.si = si;
            r.mu_w = mu_w.cast<float>();
            r.n_w = (R * s.n).cast<float>();
            r.w = static_cast<float>(s.W);
            grid[to_key(mu_w, inv_vox)].push_back(static_cast<uint32_t>(refs.size()));
            refs.push_back(r);
        }
    }

    const float min_dot = cfg_.min_nromal_dot;
    const float max_off = cfg_.max_plane_offset_m;
    const float min_ext_sq = cfg_.min_lateral_extend * cfg_.min_lateral_extend;

    std::vector<uint32_t> order;
    std::vector<uint32_t> group;
    std::unordered_map<uint32_t, Contribution> per_submap;
    uint64_t contrib_total = 0;

    for (auto& [key, cell] : grid) {
        if (cell.size() < 2) { ++st.dropped_few_submaps; continue; }

        order.assign(cell.begin(), cell.end());
        std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) {
            return refs[a].w > refs[b].w;
        });
        std::vector<uint32_t> used(order.size(), 0);

        for (size_t i = 0; i < order.size(); ++i) {
            if (used[i]) continue;
            const Ref& seed = refs[order[i]];
            group.clear();
            group.push_back(order[i]);
            used[i] = 1;

            // greedy absorb
            for (size_t j = i + 1; j < order.size(); ++j) {
                if (used[j]) continue;
                const Ref& o = refs[order[j]];
                if (o.n_w.dot(seed.n_w) < min_dot) continue;
                const float off = seed.n_w.dot(o.mu_w - seed.mu_w);
                if (std::fabs(off) > max_off) continue;
                group.push_back(order[j]);
                used[j] = 1;
            }

            //
            per_submap.clear();
            for (uint32_t gi : group) {
                const Ref& r = refs[gi];
                auto [it, inserted] = per_submap.try_emplace(r.sm);
                Contribution& c = it->second;
                if (inserted) {
                    c.pose_idx = r.sm;
                    c.W = 0.0;
                    c.S1.setZero();
                    c.S2.setZero();
                }
                const CachedSurfel& s = cache_[r.sm].surfels[r.si];
                c.W += s.W;
                c.S1 += s.S1;
                c.S2 += s.S2;
            }
            if (per_submap.size() < cfg_.min_submaps_per_cluster) {
                ++st.dropped_few_submaps;
                continue;
            }

            // validate under current poses
            double Wt = 0.0;
            Eigen::Vector3d S1t = Eigen::Vector3d::Zero();
            Eigen::Matrix3d S2t = Eigen::Matrix3d::Zero();
            for (const auto& [sm, c] : per_submap) {
                const TransformedContribution tc = transform_contribution(c, poses[sm]);
                Wt += tc.W;
                S1t += tc.S1;
                S2t += tc.S2;
            }
            if (Wt < cfg_.min_cluster_weight) { ++st.dropped_weight; continue; }
            
            const Eigen::Vector3d mu = S1t / Wt;
            const Eigen::Matrix3d A = S2t / Wt - mu * mu.transpose();
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
            es.computeDirect(A);
            const double l0 = std::max(0.0, es.eigenvalues()(0));
            const double l1 = std::max(0.0, es.eigenvalues()(1));

            if (l1 < static_cast<double>(min_ext_sq)) { ++st.dropped_planarity; continue; }
            if (l1 / (l0 + 1e-12) < static_cast<double>(cfg_.planarity_ratio)) { ++st.dropped_planarity; continue; }

            Eigen::Vector3f n_mean = Eigen::Vector3f::Zero();
            for (uint32_t gi : group) n_mean += refs[gi].w * refs[gi].n_w;
            const float nn = n_mean.norm();
            if (nn > 1e-6f) {
                const Eigen::Vector3d u = es.eigenvectors().col(0);
                const float align = std::fabs(u.cast<float>().dot(n_mean) / nn);
                if (align < cfg_.min_normal_consistency) { ++st.dropped_planarity; continue; }
            }

            Cluster cl;
            cl.contribs.reserve(per_submap.size());
            for (auto& [sm, c] : per_submap) cl.contribs.push_back(std::move(c));
            contrib_total += cl.contribs.size();
            clusters_.push_back(std::move(cl));
        }
    }

    st.clusters = static_cast<uint32_t>(clusters_.size());
    st.mean_contribs = clusters_.empty() ? 0.0f : static_cast<float>(contrib_total) / static_cast<float>(clusters_.size());
}

double BundleAdjustment::chain_cost(const std::vector<Eigen::Isometry3d>& poses) const {
    const double inv_sr = 1.0 / static_cast<double>(cfg_.odom_sigma_rot_rad);
    const double inv_st = 1.0 / static_cast<double>(cfg_.odom_sigma_trans_m);
    double c = 0.0;
    for (size_t i = 0; i + 1 < cache_.size(); ++i) {
        const Eigen::Isometry3d Z = cache_[i].T_origin.inverse() * cache_[i + 1].T_origin;
        c += chain_residual(poses[i], poses[i + 1], Z, inv_sr, inv_st).squaredNorm();
    }

    return c;
}

double BundleAdjustment::total_cost(const std::vector<Eigen::Isometry3d>& poses) const {
    double c = chain_cost(poses);
    for (const Cluster& cl : clusters_) {
        double Wt = 0.0;
        Eigen::Vector3d S1t = Eigen::Vector3d::Zero();
        Eigen::Matrix3d S2t = Eigen::Matrix3d::Zero();
        for (const Contribution& cb : cl.contribs) {
            const TransformedContribution tc = transform_contribution(cb, poses[cb.pose_idx]);
            Wt += tc.W;
            S1t += tc.S1;
            S2t += tc.S2;
        }
        c += cluster_cost(Wt, S1t, S2t);
    }
    return c;
}

int BundleAdjustment::lm_solve(std::vector<Eigen::Isometry3d>& poses, Stats& st) {
    const size_t V = poses.size();
    if (V < 2 || cfg_.max_gn_iterations <= 0) return 0;
    const int n = static_cast<int>(6 * (V - 1));
    const double h = cfg_.fd_step;
    const double inv_sr = 1.0 / static_cast<double>(cfg_.odom_sigma_rot_rad);
    const double inv_st = 1.0 / static_cast<double>(cfg_.odom_sigma_trans_m);

    auto var_base = [](uint32_t p) -> int {
        return p == 0 ? -1 : static_cast<int>(p - 1) * 6;
    };

    double mu_lm = cfg_.lm_init;
    double cost = total_cost(poses);
    int iters = 0;

    for (int it = 0; it < cfg_.max_gn_iterations; ++it) {
        Eigen::VectorXd g = Eigen::VectorXd::Zero(n);
        std::vector<Eigen::Triplet<double>> trips;
        trips.reserve(clusters_.size() * 18 * 18 + (V - 1) * 144 - n);
        for (int d = 0; d < n; ++d) trips.emplace_back(d, d, 1e-12);

        // cluster terms
        std::vector<TransformedContribution> tcs;
        for (const Cluster& cl : clusters_) {
            const size_t m = cl.contribs.size();
            tcs.clear();
            tcs.reserve(m);
            double Wt = 0.0;
            Eigen::Vector3d S1t = Eigen::Vector3d::Zero();
            Eigen::Matrix3d S2t = Eigen::Matrix3d::Zero();
            for (const Contribution& cb : cl.contribs) {
                tcs.push_back(transform_contribution(cb, poses[cb.pose_idx]));
                Wt += tcs.back().W;
                S1t += tcs.back().S1;
                S2t += tcs.back().S2;
            }
            const double l0 = cluster_cost(Wt, S1t, S2t);

            Eigen::VectorXd gamma = Eigen::VectorXd::Zero(6 * m);
            for (size_t k = 0; k < m; ++k) {
                const Contribution& cb = cl.contribs[k];
                if (var_base(cb.pose_idx) < 0) continue;

                const double Wr = Wt - tcs[k].W;
                const Eigen::Vector3d S1r = S1t - tcs[k].S1;
                const Eigen::Matrix3d S2r = S2t - tcs[k].S2;
                for (int j = 0; j < 6; ++j) {
                    const TransformedContribution tp = transform_contribution(cb, perturb_pose(poses[cb.pose_idx], j, +h));
                    const TransformedContribution tm = transform_contribution(cb, perturb_pose(poses[cb.pose_idx], j, -h));
                    const double cp = cluster_cost(Wr + tp.W, S1r + tp.S1, S2r + tp.S2);
                    const double cm = cluster_cost(Wr + tm.W, S1r + tm.S1, S2r + tm.S2);
                    gamma(static_cast<int>(6*k) + j) = (cp - cm) / (2.0 * h);
                }
            }

            const double denom = 2.0 * l0 + 1e-9;
            for (size_t ka = 0; ka < m; ++ka) {
                const int ba = var_base(cl.contribs[ka].pose_idx);
                if (ba < 0) continue;
                for (int ja = 0; ja < 6; ++ja) {
                    const double ga = gamma(static_cast<int>(6 * ka) + ja);
                    if (ga == 0.0) continue;
                    g(ba + ja) += ga;
                    for (size_t kb = 0; kb < m; ++kb) {
                        const int bb = var_base(cl.contribs[kb].pose_idx);
                        if (bb < 0) continue;
                        for (int jb = 0; jb < 6; ++jb) {
                            const double gb = gamma(static_cast<int>(6 * kb) + jb);
                            if (gb == 0.0) continue;
                            trips.emplace_back(ba + ja, bb + jb, ga * gb / denom);
                        }
                    }
                }
            }
        }

        // odometry chain terms
        for (size_t i = 0; i + 1 < V; ++i) {
            const Eigen::Isometry3d Z = cache_[i].T_origin.inverse() * cache_[i + 1].T_origin;
            const Eigen::Matrix<double, 6, 1> r = chain_residual(poses[i], poses[i+1], Z, inv_sr, inv_st);

            Eigen::Matrix<double, 6, 12> J = Eigen::Matrix<double, 6, 12>::Zero();
            for (int side = 0; side < 2; ++side) {
                const size_t p = i + side;
                if (var_base(static_cast<uint32_t>(p)) < 0) continue;
                for (int j = 0; j < 6; ++j) {
                    Eigen::Isometry3d Tp = poses[p];
                    Eigen::Isometry3d Tm = poses[p];
                    Tp = perturb_pose(Tp, j, +h);
                    Tm = perturb_pose(Tm, j, -h);
                    const auto rp = (side == 0) 
                        ? chain_residual(Tp, poses[i + 1], Z, inv_sr, inv_st)
                        : chain_residual(poses[i], Tp, Z, inv_sr, inv_st);
                    const auto rm = (side == 0)
                        ? chain_residual(Tm, poses[i + 1], Z, inv_sr, inv_st)
                        : chain_residual(poses[i], Tm, Z, inv_sr, inv_st);
                    J.col(side * 6 + j) = (rp - rm) / (2.0 * h);
                }
            }

            const Eigen::Matrix<double, 12, 1> gj = 2.0 * J.transpose() * r;
            const Eigen::Matrix<double, 12, 12> Hj = 2.0 * J.transpose() * J;
            const int bases[2] = { var_base(static_cast<uint32_t>(i)), var_base(static_cast<uint32_t>(i + 1)) };
            for (int sa = 0; sa < 2; ++sa) {
                if (bases[sa] < 0) continue;
                for (int ja = 0; ja < 6; ++ja) {
                    g(bases[sa] + ja) += gj(sa * 6 + ja);
                    for (int sb = 0; sb < 2; ++sb) {
                        if (bases[sb] < 0) continue;
                        for (int jb = 0; jb < 6; ++jb) {
                            const double v = Hj(sa * 6 + ja, sb * 6 + jb);
                            if (v != 0.0) trips.emplace_back(bases[sa] + ja, bases[sb] + jb, v);
                        }
                    }
                }
            }
        }

        Eigen::SparseMatrix<double> H(n,n);
        H.setFromTriplets(trips.begin(), trips.end());

        // LM accept loop
        bool accepted = false;
        for (int attempt = 0; attempt < 6 && !accepted; ++attempt) {
            Eigen::SparseMatrix<double> Hlm = H;
            for (int d = 0; d < n; ++d) {
                const double diag = H.coeff(d,d);
                Hlm.coeffRef(d, d) = diag + mu_lm * std::max(diag, 1e-9);
            }

            Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
            solver.compute(Hlm);
            if (solver.info() != Eigen::Success) { mu_lm *= 10.0; continue; }
            const Eigen::VectorXd delta = solver.solve(-g);
            if (solver.info() != Eigen::Success) { mu_lm *= 10.0; continue; }

            std::vector<Eigen::Isometry3d> cand = poses;
            for (size_t p = 1; p < V; ++p) {
                apply_delta(cand[p], delta.segment<6>(static_cast<int>(6 * (p - 1))));
            }

            const double cnew = total_cost(cand);
            if (cnew < cost) {
                const double rel = (cost - cnew) / std::max(cost, 1e-12);
                poses = std::move(cand);
                cost = cnew;
                mu_lm = std::max(mu_lm / 3.0, 1e-12);
                accepted = true;
                ++iters;
                if (rel < cfg_.min_rel_cost_decrease || delta.lpNorm<Eigen::Infinity>() < cfg_.min_step_norm) {
                    st.iterations += iters;
                    return iters;
                }
            }
            else {
                mu_lm *= 10.0;
            }
        }

        if (!accepted) break; // dampening exhausted: at (local) optimum
    }

    st.iterations += iters;
    return iters;
}

BundleAdjustment::Result BundleAdjustment::optimize(const MapSnapshot& snap) {
    Result res;
    if (cache_.size() < 2) return res;

    std::unordered_map<SubmapId, Eigen::Isometry3d> snap_pose;
    snap_pose.reserve(snap.views.size());
    for (const auto& v : snap.views) {
        snap_pose.emplace(v.id, v.T_submap_world.cast<double>());
    }

    std::vector<Eigen::Isometry3d> poses(cache_.size());
    for (size_t i = 0; i < cache_.size(); ++i) {
        const auto it = snap_pose.find(cache_[i].id);
        poses[i] = (it != snap_pose.end()) ? it->second : cache_[i].T_origin;
    }

    for (int outer = 0; outer < cfg_.max_outer_loops; ++outer) {
        ++res.stats.outer_loops;
        const std::vector<Eigen::Isometry3d> before = poses;

        double t0 = now_ms();
        associate(poses, res.stats);
        res.stats.t_associate_ms += now_ms() - t0;

        if (outer == 0) res.stats.cost_initial = total_cost(poses);

        t0 = now_ms();
        lm_solve(poses, res.stats);
        res.stats.t_solve_ms += now_ms() - t0;

        double max_change = 0.0;
        for (size_t i = 0; i < poses.size(); ++i) {
            max_change = std::max(max_change, (poses[i].translation() - before[i].translation()).norm());
            max_change = std::max(max_change, so3_log(before[i].rotation().transpose() * poses[i].rotation()).norm());
        }
        if (max_change < cfg_.reassoc_min_pose_change) break;
    }

    res.stats.cost_final = total_cost(poses);
    for (size_t i = 0; i < cache_.size(); ++i) {
        PoseCorrection pc;
        pc.id = cache_[i].id;
        pc.T_submap_world_corrected = poses[i].cast<float>();
        res.corrections.push_back(pc);
    }
    
    return res;
}

} // namespace smip_uav