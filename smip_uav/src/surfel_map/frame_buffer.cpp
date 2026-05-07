#include "surfel_map/frame_buffer.hpp"
#include <algorithm>
#include <numeric>

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
        build_tracks();
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
    c.surfels.reserve(N);
    c.track_ids.reserve(N);
    c.track_sizes.reserve(N);

    for (size_t i = 0; i < N; ++i) {
        const int32_t tid = oldest.track_ids[i];
        const uint8_t tsz = (tid < 0) ? uint8_t{1} : static_cast<uint8_t>(track_size_.count(tid) ? track_size_[tid] : uint8_t{1});

        if (tsz < cfg_.M_min) continue; // gate: Surfels has to be tracked for M_min frames

        c.surfels.push_back(std::move(oldest.surfels[i]));
        c.track_ids.push_back(tid);
        c.track_sizes.push_back(tsz);
    }

    slots_.pop_front();
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
    if (slots_.empty()) return;

    // refresh world-frame caches and hashes 
    for (auto& bf : slots_) {
        if (bf.cache_dirty) rebuild_frame_cache(bf);
    }

    // reset per-frame track ids; flat node table for union find
    std::vector<size_t> frame_offsets(slots_.size() + 1, 0);
    for (size_t i = 0; i < slots_.size(); ++i) {
        slots_[i].track_ids.assign(slots_[i].surfels.size(), -1);
        frame_offsets[i + 1] = frame_offsets[i] + slots_[i].surfels.size();
    }
    const size_t total_nodes = frame_offsets.back();
    if (total_nodes == 0) return;
    UnionFind uf(total_nodes);

    // pairwise matching with mutual-best on
    const float int_vs = 1.0f / cfg_.voxel_size;

    auto match_pair = [&](size_t i, size_t j) {
        const BufferFrame& fa = slots_[i];
        const BufferFrame& fb = slots_[j];
        const size_t Na = fa.surfels.size();
        const size_t Nb = fb.surfels.size();
        if (Na == 0 || Nb == 0) return;

        std::vector<int32_t> best_b_for_a(Na, -1);
        std::vector<float> score_a(Na, std::numeric_limits<float>::max());
        std::vector<int32_t> best_a_for_b(Nb, -1);
        std::vector<float> score_b(Nb, std::numeric_limits<float>::max());

        for (size_t ka = 0; ka < Na; ++ka) {
            const Eigen::Vector3f& mu_a = fa.mu_w[ka];
            const Eigen::Vector3f& n_a = fa.n_w[ka];
            const Eigen::Matrix3f& S_a = fa.S_w[ka];

            const VoxelKey kc = to_key(mu_a, int_vs);

            for (const auto& o : kNb7) {
                const VoxelKey k{kc.x + o[0], kc.y + o[1], kc.z + o[2]};
                auto it = fb.voxel_index.find(k);
                if (it == fb.voxel_index.end()) continue;

                for (uint16_t kb_u : it->second) {
                    const size_t kb = kb_u;

                    const float cos_n = n_a.dot(fb.n_w[kb]);
                    if (cos_n < cfg_.corr_normal_cos) continue;

                    const Eigen::Vector3f d = mu_a - fb.mu_w[kb];
                    const Eigen::Matrix3f S = S_a + fb.S_w[kb];
                    const float d2 = d.dot(S.ldlt().solve(d));
                    if (!std::isfinite(d2) || d2 < 0.0f) continue;
                    if (d2 >= cfg_.corr_mahal_sq) continue;

                    if (d2 < score_a[ka]) {
                        score_a[ka] = d2;
                        best_b_for_a[ka] = static_cast<int32_t>(kb);
                    }
                    if (d2 < score_b[kb]) {
                        score_b[kb] = d2;
                        best_a_for_b[kb] = static_cast<int32_t>(ka);
                    }
                }
            }
        }
        // mutual best filter
        const int32_t off_a = static_cast<int32_t>(frame_offsets[i]);
        const int32_t off_b = static_cast<int32_t>(frame_offsets[j]);
        for(size_t ka = 0; ka < Na; ++ka) {
            const int32_t kb = best_b_for_a[ka];
            if (kb < 0) continue;
            if (best_a_for_b[kb] != static_cast<int32_t>(ka)) continue;
            uf.unite(off_a + static_cast<int32_t>(ka), off_b + kb);
        }
    };

    for (size_t i = 0; i+1 < slots_.size(); ++i) {
        for (size_t j = i+1; j < slots_.size(); ++j) {
            match_pair(i, j);
        }
    }

    // resolve canonical track ids; populate track_size_ for evict-time queries
    for (size_t i = 0; i < slots_.size(); ++i) {
        auto& bf = slots_[i];
        for (size_t k = 0; k < bf.surfels.size(); ++k) {
            const int32_t node = static_cast<int32_t>(frame_offsets[i] + k);
            const int32_t root = uf.find(node);
            const uint16_t sz = uf.size_of(root);
            if (sz < 2) continue;
            bf.track_ids[k] = root;
            track_size_[root] = static_cast<uint8_t>(std::min<uint16_t>(sz, 255));
        }
    }

    return;
}

bool FrameBuffer::track_confirmed(int32_t track_id) const {
    if (track_id < 0) return false;
    auto it = track_size_.find(track_id);
    if (it == track_size_.end()) return false;
    return it->second >= cfg_.M_min;
}

void FrameBuffer::run_ba() {
    //todo
    return;
}

}