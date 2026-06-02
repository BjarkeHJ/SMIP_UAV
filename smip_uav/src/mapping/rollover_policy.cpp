#include "mapping/rollover_policy.hpp"

namespace smip_uav {

RolloverSignal RolloverPolicy::evaluate(const StampedPose& current_pose, const ActiveMap& submap, const MapStateContainer& container) const {
    RolloverSignal sig;

    if (!threshold_exceeded(submap)) {
        return sig;
    }

    const Eigen::Vector3f p_world = current_pose.T_world.translation();

    const auto overlap_id = container.query_overlap(p_world);

    sig.action = RolloverAction::ROLLOVER;
    if (overlap_id.has_value()) {
        sig.action = RolloverAction::LOOP_CLOSURE_HINT;
        sig.overlap_id = overlap_id;
    }

    return sig;
}

bool RolloverPolicy::threshold_exceeded(const ActiveMap& submap) const {
    if (submap.frame_count() >= cfg_.max_frames) return true;
    if (submap.accumulated_translation() >= cfg_.max_translation_m) return true;
    if (submap.accumulated_rotation() >= cfg_.max_rotation_rad) return true;
    return false;
}



} // namespace smip_uav