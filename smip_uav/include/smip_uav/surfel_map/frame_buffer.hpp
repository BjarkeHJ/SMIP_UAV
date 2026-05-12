#ifndef FRAME_BUFFER_HPP_
#define FRAME_BUFFER_HPP_

#include <unordered_set>
#include <deque>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "common/point_types.hpp"
#include "surfel_map/voxel_grid.hpp"

namespace smip_uav {

struct CommittedSurfels {
    std::vector<FrameSurfel> surfels;
    std::vector<int32_t> track_ids;
    std::vector<uint8_t> track_sizes;
    std::vector<uint8_t> is_fused;
    Eigen::Isometry3f pose{Eigen::Isometry3f::Identity()};
    int64_t timestamp{0};
    uint64_t frame_id{0};
    size_t original_count{0};
};

struct TrackedSurfelViz {
    Eigen::Vector3f position_w;
    int32_t track_id{-1};    // -1 = untracked
    uint8_t track_size{0};   // 0 = untracked, otherwise frames spanned
    uint8_t frame_slot{0};   // index into the sliding window (0 = oldest)
};

class FrameBuffer {
public:
    struct Config {
        size_t window_size{10};

        float voxel_size{0.25f};
        float corr_normal_cos{0.95f};
        float corr_mahal_sq{2.0f};
        size_t M_min{5};

        bool enable_ba{false};
        size_t ba_max_iters{3};
    };


    FrameBuffer() = default;
    explicit FrameBuffer(const Config& cfg);

    std::vector<CommittedSurfels> push(std::vector<FrameSurfel> surfels, const Eigen::Isometry3f& pose, int64_t timestamp);
    std::vector<CommittedSurfels> flush();

    std::vector<TrackedSurfelViz> get_buffer_viz() const;

    size_t size() const { return slots_.size(); }
    bool full() const { return slots_.size() >= cfg_.window_size; }
    bool empty() const { return slots_.empty(); }

    size_t active_track_count() const { return track_size_.size(); }

private:
    using SurfelIndexHash = std::unordered_map<VoxelKey, std::vector<uint16_t>, VoxelKeyHash>;
    using TrackMember = std::pair<uint64_t, size_t>;  // (frame_id, surfel_idx)

    struct BufferFrame {
        std::vector<FrameSurfel> surfels;
        Eigen::Isometry3f pose{Eigen::Isometry3f::Identity()};
        int64_t timestamp{0};
        uint64_t frame_id{0};

        std::vector<int32_t> track_ids;
    
        std::vector<Eigen::Vector3f> mu_w;
        std::vector<Eigen::Vector3f> n_w;
        std::vector<Eigen::Matrix3f> S_w;
        SurfelIndexHash voxel_index;

        bool cache_dirty{true};
    };

    CommittedSurfels evict_oldest();

    void build_tracks();
    void rebuild_frame_cache(BufferFrame& bf);
    bool track_confirmed(int32_t track_id) const;

    void run_ba();

    Config cfg_;
    std::deque<BufferFrame> slots_;
    uint64_t next_frame_id_{1};

    std::unordered_map<int32_t, uint8_t> track_size_;
    std::unordered_map<int32_t, std::vector<TrackMember>> track_members_;

};

} // namespace smip_uav

#endif