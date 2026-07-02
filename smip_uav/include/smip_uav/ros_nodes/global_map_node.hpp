#ifndef SMIP_GLOBAL_MAP_NODE_HPP_
#define SMIP_GLOBAL_MAP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>

#include "mapping/map_state_container.hpp"
#include "mapping/bundle_adjustment.hpp"
#include "utils/stop_watch.hpp"

namespace smip_uav {

class GlobalMapNode : public rclcpp::Node {
public:
    explicit GlobalMapNode(std::shared_ptr<MapStateContainer> container);

private:
    void opt_cycle();
    void publish_pose_graph();
    void publish_surfel_points();
    void publish_surfel_markers();

    std::shared_ptr<MapStateContainer> map_state_container_;
    std::unique_ptr<BundleAdjustment> ba_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surfel_point_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr surfel_marker_pub_;

    double opt_period_s_{1.0};

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    std::string map_frame_{"map"};
    size_t last_submap_count_{0};

    Eigen::Isometry3f T_map_odom_{Eigen::Isometry3f::Identity()};
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace smip_uav

#endif