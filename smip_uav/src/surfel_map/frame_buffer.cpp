#include "surfel_map/frame_buffer.hpp"
#include <algorithm>
#include <numeric>
#include <iostream>

namespace smip_uav {

namespace {

struct UnionFind {
    std::vector<int32_t> parent;
    std::vector<int32_t> rank_;
    std::vector<uint16_t> size_;

    explicit UnionFind(size_t n) : parent(n), rank_(n, 0), size_(n, 1) 
    {
        std::iota(parent.begin(), parent.end(), 0);
    }

    int32_t find(int32_t x) {
        while(parent[x] != x) {
            parent[x] = parent[parent[x]]; // path compression
            x = parent[x];
        }
        return x;
    }

    void unite(int32_t a, int32_t b) {
        a = find(a);
        b = find(b);
        if (a == b) return;
        if (rank_[a] < rank_[b]) std::swap(a, b);
        parent[b] = a;
        size_[a] = static_cast<uint16_t>(size_[a] + size_[b]);
        if (rank_[a] == rank_[b]) ++rank_[a];
    }

    uint16_t size_of(int32_t x) { return size_[find(x)]; }
};

constexpr int32_t kNb7[7][3] = {
    {0, 0, 0},
    {-1, 0, 0}, {1, 0, 0},
    {0, -1, 0}, {0, 1, 0},
    {0, 0, -1}, {0, 0, 1}
};

inline VoxelKey to_key(const Eigen::Vector3f& p, float inv_vs) {
    return {
        static_cast<int32_t>(std::floor(p.x() * inv_vs)),
        static_cast<int32_t>(std::floor(p.y() * inv_vs)),
        static_cast<int32_t>(std::floor(p.z() * inv_vs))
    };
}

constexpr float kFuseAlpha = 0.005f; // ToF depth noise coeff est.

} // anonymous namespace


FrameBuffer::FrameBuffer(const Config& cfg) : cfg_(cfg) {}

std::vector<CommittedSurfels> FrameBuffer::push(std::vector<FrameSurfel> surfels, const Eigen::Isometry3f& pose, int64_t timestamp) {

    // Insert new frame in the buffer
    BufferFrame bf;
    bf.surfels = std::move(surfels);
    bf.pose = pose;
    bf.timestamp = timestamp;
    bf.frame_id = next_frame_id_++;
    bf.track_ids.assign(bf.surfels.size(), -1);
    bf.cache_dirty = true;
    slots_.push_back(std::move(bf));

    // TODO: track surfel and do bundle adjustments
    build_tracks();
    if (cfg_.enable_ba) {
        run_ba();
        for (auto& s : slots_) s.cache_dirty = true;
        cached_edge_pairs_.clear(); // poses changed — all cached matches are stale
        build_tracks(); // rebuild tracks after ba
    }
    
    // Overflowing buffer -> return oldest
    std::vector<CommittedSurfels> out;
    if (slots_.size() > cfg_.window_size) {
        out.push_back(evict_oldest());
    }
    return out;
}

std::vector<CommittedSurfels> FrameBuffer::flush() {
    std::vector<CommittedSurfels> out;
    out.reserve(slots_.size());
    while(!slots_.empty()) {
        out.push_back(evict_oldest());
    }
    return out;
}

CommittedSurfels FrameBuffer::evict_oldest() {
    BufferFrame& oldest = slots_.front();
    const size_t N = oldest.surfels.size();

    CommittedSurfels c;
    c.pose = oldest.pose;
    c.timestamp = oldest.timestamp;
    c.frame_id = oldest.frame_id;
    c.original_count = N;
    c.surfels.reserve(N);
    c.track_ids.reserve(N);
    c.track_sizes.reserve(N);

    for (size_t i = 0; i < N; ++i) {
        const int32_t tid = oldest.track_ids[i];
        const auto it = (tid >= 0) ? track_size_.find(tid) : track_size_.end();
        const uint8_t tsz = (it != track_size_.end()) ? it->second : uint8_t{1};

        if (tsz < cfg_.M_min) continue; // gate: Surfels has to be tracked for M_min frames

        c.surfels.push_back(std::move(oldest.surfels[i]));
        c.track_ids.push_back(tid);
        c.track_sizes.push_back(tsz);
        c.is_fused.push_back(0);
    }

    slots_.pop_front();
    if (!cached_edge_pairs_.empty())
        cached_edge_pairs_.pop_front();
    return c;
}

void FrameBuffer::rebuild_frame_cache(BufferFrame& bf) {
    const size_t N = bf.surfels.size();
    bf.mu_w.resize(N);
    bf.n_w.resize(N);
    bf.S_w.resize(N);
    bf.voxel_index.clear();

    const Eigen::Matrix3f R = bf.pose.rotation();
    const float inv_vs = 1.0f / cfg_.voxel_size;

    for (size_t k = 0; k < N; ++k) {
        const auto& s = bf.surfels[k];
        bf.mu_w[k] = bf.pose * s.centroid;
        bf.n_w[k] = R * s.normal;
        bf.S_w[k] = R * s.C_shape * R.transpose();

        const VoxelKey key = to_key(bf.mu_w[k], inv_vs);
        bf.voxel_index[key].push_back(static_cast<uint16_t>(k));
    }

    bf.cache_dirty = false;
}

void FrameBuffer::build_tracks() {
    track_size_.clear();
    track_members_.clear();
    if (slots_.empty()) return;

    // Rebuild dirty caches (only the new frame is dirty in steady state)
    for (auto& bf : slots_) {
        if (bf.cache_dirty) rebuild_frame_cache(bf);
    }

    // Fill any missing edge pairs. In steady state this runs exactly once for the
    // newest pair. After a full cache invalidation (e.g. post-BA) it runs for all pairs.
    while (cached_edge_pairs_.size() < slots_.size() - 1) {
        const size_t i = cached_edge_pairs_.size(); // index of next uncached pair
        cached_edge_pairs_.push_back(compute_matches(i, i + 1));
    }

    // Reset per-frame track IDs and build flat node offset table
    std::vector<size_t> frame_offsets(slots_.size() + 1, 0);
    for (size_t i = 0; i < slots_.size(); ++i) {
        slots_[i].track_ids.assign(slots_[i].surfels.size(), -1);
        frame_offsets[i + 1] = frame_offsets[i] + slots_[i].surfels.size();
    }
    const size_t total_nodes = frame_offsets.back();
    if (total_nodes == 0) return;
    UnionFind uf(total_nodes);

    // Replay all cached edges into the UnionFind.
    // Invariant: cached_edge_pairs_[p] holds matches between slots_[p] and slots_[p+1].
    for (size_t p = 0; p < cached_edge_pairs_.size(); ++p) {
        const int32_t off_a = static_cast<int32_t>(frame_offsets[p]);
        const int32_t off_b = static_cast<int32_t>(frame_offsets[p + 1]);
        for (const auto& e : cached_edge_pairs_[p]) {
            uf.unite(off_a + e.idx_a, off_b + e.idx_b);
        }
    }

    // Resolve canonical track IDs; count distinct frames per component via bitmask.
    // window_size is always <= 64 so uint64_t suffices.
    std::unordered_map<int32_t, uint64_t> component_frame_mask;
    for (size_t i = 0; i < slots_.size(); ++i) {
        const uint64_t bit = uint64_t{1} << i;
        for (size_t k = 0; k < slots_[i].surfels.size(); ++k) {
            const int32_t node = static_cast<int32_t>(frame_offsets[i] + k);
            const int32_t root = uf.find(node);
            if (uf.size_of(root) < 2) continue;
            component_frame_mask[root] |= bit;
        }
    }

    for (size_t i = 0; i < slots_.size(); ++i) {
        auto& bf = slots_[i];
        for (size_t k = 0; k < bf.surfels.size(); ++k) {
            const int32_t node = static_cast<int32_t>(frame_offsets[i] + k);
            const int32_t root = uf.find(node);
            if (uf.size_of(root) < 2) continue;

            bf.track_ids[k] = root;
            track_size_.emplace(root, static_cast<uint8_t>(__builtin_popcountll(component_frame_mask[root])));
            track_members_[root].emplace_back(slots_[i].frame_id, k);
        }
    }
}

std::vector<FrameBuffer::MatchEdge> FrameBuffer::compute_matches(size_t i, size_t j) {
    const BufferFrame& fa = slots_[i];
    const BufferFrame& fb = slots_[j];
    const size_t Na = fa.surfels.size();
    const size_t Nb = fb.surfels.size();

    std::vector<MatchEdge> result;
    if (Na == 0 || Nb == 0) return result;

    std::vector<int32_t> best_b_for_a(Na, -1);
    std::vector<float>   score_a(Na, std::numeric_limits<float>::max());
    std::vector<int32_t> best_a_for_b(Nb, -1);
    std::vector<float>   score_b(Nb, std::numeric_limits<float>::max());

    const float inv_vs  = 1.0f / cfg_.voxel_size;
    const float eucl_sq = cfg_.voxel_size * cfg_.voxel_size;

    for (size_t ka = 0; ka < Na; ++ka) {
        const Eigen::Vector3f& mu_a = fa.mu_w[ka];
        const Eigen::Vector3f& n_a  = fa.n_w[ka];
        const Eigen::Matrix3f& S_a  = fa.S_w[ka];

        const VoxelKey kc = to_key(mu_a, inv_vs);

        for (const auto& o : kNb7) {
            const VoxelKey k{kc.x + o[0], kc.y + o[1], kc.z + o[2]};
            auto it = fb.voxel_index.find(k);
            if (it == fb.voxel_index.end()) continue;

            for (uint16_t kb_u : it->second) {
                const size_t kb = kb_u;

                const float cos_n = n_a.dot(fb.n_w[kb]);
                if (cos_n < cfg_.corr_normal_cos) continue;

                const Eigen::Vector3f d = mu_a - fb.mu_w[kb];
                if (d.squaredNorm() > eucl_sq) continue;

                const Eigen::Matrix3f S = S_a + fb.S_w[kb];
                const float d2 = d.dot(S.inverse() * d);
                if (!std::isfinite(d2) || d2 < 0.0f) continue;
                if (d2 >= cfg_.corr_mahal_sq) continue;

                if (d2 < score_a[ka]) { score_a[ka] = d2; best_b_for_a[ka] = static_cast<int32_t>(kb); }
                if (d2 < score_b[kb]) { score_b[kb] = d2; best_a_for_b[kb] = static_cast<int32_t>(ka); }
            }
        }
    }

    // Mutual-best filter
    for (size_t ka = 0; ka < Na; ++ka) {
        const int32_t kb = best_b_for_a[ka];
        if (kb < 0) continue;
        if (best_a_for_b[kb] != static_cast<int32_t>(ka)) continue;
        result.push_back({static_cast<uint16_t>(ka), static_cast<uint16_t>(kb)});
    }
    return result;
}

std::vector<TrackedSurfelViz> FrameBuffer::get_buffer_viz() const {
    std::vector<TrackedSurfelViz> out;
    for (size_t slot = 0; slot < slots_.size(); ++slot) {
        const BufferFrame& bf = slots_[slot];
        if (bf.cache_dirty) continue;
        for (size_t k = 0; k < bf.mu_w.size(); ++k) {
            TrackedSurfelViz v;
            v.position_w = bf.mu_w[k];
            v.frame_slot = static_cast<uint8_t>(slot);
            const int32_t tid = bf.track_ids[k];
            if (tid < 0) {
                v.track_id   = -1;
                v.track_size = 0;
            } else {
                auto sz_it = track_size_.find(tid);
                v.track_size = (sz_it != track_size_.end()) ? sz_it->second : uint8_t{1};

                // Stable color seed: anchor on the oldest frame_id + surfel index in
                // this track so the color doesn't change as the union-find root shifts.
                auto mem_it = track_members_.find(tid);
                if (mem_it != track_members_.end() && !mem_it->second.empty()) {
                    // Anchor on the oldest frame_id in this track — frame_ids are
                    // assigned once and never shift, so this is stable across evictions.
                    uint64_t oldest_fid = std::numeric_limits<uint64_t>::max();
                    size_t   oldest_idx = 0;
                    for (const auto& [fid, idx] : mem_it->second) {
                        if (fid < oldest_fid) { oldest_fid = fid; oldest_idx = idx; }
                    }
                    // Wang hash — integer-only, no float precision loss
                    uint32_t h = static_cast<uint32_t>(oldest_fid * 65537ULL + oldest_idx);
                    h ^= h >> 16; h *= 0x45d9f3bU; h ^= h >> 16;
                    v.track_id = static_cast<int32_t>(h & 0x7FFFFFFFu);
                } else {
                    v.track_id = static_cast<int32_t>(tid);
                }
            }
            out.push_back(v);
        }
    }
    return out;
}

bool FrameBuffer::track_confirmed(int32_t track_id) const {
    if (track_id < 0) return false;
    auto it = track_size_.find(track_id);
    if (it == track_size_.end()) return false;
    return it->second >= cfg_.M_min;
}

void FrameBuffer::run_ba() {
    if (slots_.size() < 2) return;

    BufferFrame& oldest = slots_.front();
    const uint64_t oldest_fid = oldest.frame_id;
    const Eigen::Isometry3f T_prior = oldest.pose;

    // Build frame_id -> slot index for anchor lookups
    std::unordered_map<uint64_t, size_t> fid_to_slot;
    fid_to_slot.reserve(slots_.size());
    for (size_t s = 0; s < slots_.size(); ++s)
        fid_to_slot[slots_[s].frame_id] = s;

    for (size_t iter = 0; iter < cfg_.ba_max_iters; ++iter) {
        Eigen::Matrix<float, 6, 6> H = Eigen::Matrix<float, 6, 6>::Zero();
        Eigen::Matrix<float, 6, 1> b = Eigen::Matrix<float, 6, 1>::Zero();

        const Eigen::Matrix3f R0 = oldest.pose.rotation();

        for (size_t i = 0; i < oldest.surfels.size(); ++i) {
            const int32_t tid = oldest.track_ids[i];
            if (!track_confirmed(tid)) continue;

            const auto mem_it = track_members_.find(tid);
            if (mem_it == track_members_.end()) continue;

            const FrameSurfel& si = oldest.surfels[i];
            const Eigen::Vector3f mu_w  = oldest.pose * si.centroid;
            const Eigen::Matrix3f S_0_w = R0 * si.C_shape * R0.transpose();

            for (const auto& [anchor_fid, anchor_idx] : mem_it->second) {
                if (anchor_fid == oldest_fid) continue;

                const auto slot_it = fid_to_slot.find(anchor_fid);
                if (slot_it == fid_to_slot.end()) continue;

                const BufferFrame& abf = slots_[slot_it->second];
                if (abf.cache_dirty) continue;

                const Eigen::Vector3f& n_j  = abf.n_w[anchor_idx];
                const Eigen::Vector3f& mu_j = abf.mu_w[anchor_idx];
                const Eigen::Matrix3f& S_j  = abf.S_w[anchor_idx];

                const float r = n_j.dot(mu_w - mu_j);

                // Project combined anisotropic covariance onto normal axis
                const float sigma2 = n_j.dot((S_0_w + S_j) * n_j);
                if (sigma2 < 1e-10f) continue;
                const float w = 1.0f / sigma2;

                // J = [n_j^T,  (mu_w x n_j)^T]   (left SE3 perturbation)
                Eigen::Matrix<float, 1, 6> J;
                J.head<3>() = n_j.transpose();
                J.tail<3>() = mu_w.cross(n_j).transpose();

                H.noalias() += w * J.transpose() * J;
                b.noalias() += w * J.transpose() * r;
            }
        }

        // Tikhonov regularisation in case of rank-deficient geometry
        H.diagonal().array() += 1e-6f;

        const Eigen::Matrix<float, 6, 1> dx = H.ldlt().solve(-b);

        // SE3 left update: T_new = Exp(dx) * T_cur
        const Eigen::Vector3f dphi = dx.tail<3>();
        const float angle = dphi.norm();
        Eigen::Isometry3f dT = Eigen::Isometry3f::Identity();
        if (angle > 1e-8f)
            dT.linear() = Eigen::AngleAxisf(angle, dphi / angle).toRotationMatrix();
        dT.translation() = dx.head<3>();

        oldest.pose = dT * oldest.pose;

        if (dx.norm() < 1e-5f) {
            break;
        };
    }

    const Eigen::Isometry3f delta = oldest.pose * T_prior.inverse();
    const float dt_m   = delta.translation().norm();
    const float dR_deg = Eigen::AngleAxisf(delta.rotation()).angle() * (180.0f / M_PI);
    std::printf("[run_ba] frame %lu: dt=%.4f m  dR=%.4f deg\n", oldest_fid, dt_m, dR_deg);
}

}