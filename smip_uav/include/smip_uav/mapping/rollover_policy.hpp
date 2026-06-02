#ifndef SMIP_ROLLOVER_POLICY_HPP_
#define SMIP_ROLLOVER_POLICY_HPP_

#include <optional>
#include <cstdint>

#include "core/types.hpp"
#include "mapping/active_map.hpp"
#include "mapping/map_state_container.hpp"

namespace smip_uav {

enum class RolloverAction {
    CONTINUE,
    ROLLOVER,
    LOOP_CLOSURE_HINT,
};

struct RolloverSignal {
    RolloverAction action{RolloverAction::CONTINUE};
    std::optional<SubmapId> overlap_id;
};

class RolloverPolicy {
public:
    struct Config {
        float max_translation_m{2.0f};
        // float max_rotation_rad{1.05f}; // ~60 deg acc
        float max_rotation_rad{0.52f}; // ~30 deg acc
        uint32_t max_frames{50}; // should maybe depend on fps of sensor (10Hz -> 5 sec?)

        float overlap_margin_m{0.5f}; // Checking for FrozenSubmap overlap - expand boundign sphere by this margin...

        float min_info_gain_rate{0.0f}; // future: saturation trgiger
        float degeneracy_threshold{0.0f}; // future: from pose estimator
    };

    RolloverPolicy() : RolloverPolicy(Config{}) {}
    explicit RolloverPolicy(const Config& cfg) : cfg_(cfg) {}

    RolloverSignal evaluate(const StampedPose& current_pose, const ActiveMap& submap, const MapStateContainer& container) const;

private:
    bool threshold_exceeded(const ActiveMap& submap) const;
    Config cfg_;
};


} // namespace smip_uav

#endif