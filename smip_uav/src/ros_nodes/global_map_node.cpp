#include "ros_nodes/global_map_node.hpp"
#include <geometry_msgs/msg/point.hpp>

#include <cmath>
#include <unordered_map>
#include <Eigen/Eigenvalues>

#include "core/surfel.hpp"

namespace smip_uav {

GlobalMapNode::GlobalMapNode(std::shared_ptr<MapStateContainer> container) : Node("global_map_node"), map_state_container_(container) {
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(opt_period_s_),
        std::bind(&GlobalMapNode::opt_cycle, this),
        cb_group_
    );

    RCLCPP_INFO(this->get_logger(), "GlobalMapNode ready!");
}

void GlobalMapNode::opt_cycle() {
    return;
}

} // namespace smip_uav