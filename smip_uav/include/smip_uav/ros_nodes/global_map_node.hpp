#ifndef SMIP_GLOBAL_MAP_NODE_HPP_
#define SMIP_GLOBAL_MAP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "mapping/map_state_container.hpp"

namespace smip_uav {

class GlobalMapNode : public rclcpp::Node {
public:
    explicit GlobalMapNode(std::shared_ptr<MapStateContainer> container);

private:
    void opt_cycle();
    
    std::shared_ptr<MapStateContainer> map_state_container_;

    double opt_period_s_{2.0};
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    size_t last_submap_count_{0};

};

} // namespace smip_uav

#endif