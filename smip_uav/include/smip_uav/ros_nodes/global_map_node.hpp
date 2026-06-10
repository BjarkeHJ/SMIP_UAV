#ifndef SMIP_GLOBAL_MAP_NODE_HPP_
#define SMIP_GLOBAL_MAP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>

#include "mapping/map_state_container.hpp"
#include "mapping/pose_graph.hpp"
#include "utils/stop_watch.hpp"

namespace smip_uav {

class GlobalMapNode : public rclcpp::Node {
public:
    explicit GlobalMapNode(std::shared_ptr<MapStateContainer> container);

private:
    void opt_cycle();
    void publish_pose_graph(const MapSnapshot& snap) const;
    void publish_global_map(const MapSnapshot& snap, bool full_republish);
    
    std::shared_ptr<MapStateContainer> map_state_container_;
    std::unique_ptr<PoseGraph> pose_graph_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_map_pub_;

    double opt_period_s_{2.0};
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    size_t last_submap_count_{0};
    int next_global_map_marker_id_{0};

    StopWatch sw_;
};

} // namespace smip_uav

#endif