#ifndef SMIP_ACTIVE_MAP_NODE_HPP_
#define SMIP_ACTIVE_MAP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <deque>
#include <optional>
#include <omp.h>

#include "core/frame.hpp"
#include "core/types.hpp"
#include "mapping/frame_processor.hpp"
#include "mapping/surfel_extractor.hpp"
#include "mapping/active_map.hpp"
#include "mapping/map_state_container.hpp"
#include "mapping/rollover_policy.hpp"

#include "utils/stop_watch.hpp"

namespace smip_uav {

struct XYZOffsets {
    uint32_t x{0}, y{0}, z{0};
    bool valid{false};
};

class ActiveMapNode : public rclcpp::Node {
public:
    explicit ActiveMapNode(std::shared_ptr<MapStateContainer> container);

private:
    void pointcloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    // void pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);
    void pose_callback(px4_msgs::msg::VehicleOdometry::SharedPtr pose_msg);

    void convert_pointcloud_message(sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg, Frame& frame);
    std::optional<StampedPose> get_current_pose(int64_t scan_stamp) const;
    void handle_rollover(const StampedPose& pose, const RolloverSignal& signal, int64_t stamp_ns);

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr pose_sub_;

    // Visualization
    void publish_frame_points(const Frame& frame) const;
    void publish_frame(const Frame& frame) const;
    void publish_active_map_points(const ActiveMap& map, int64_t stamp_ns) const;
    void publish_submap_surfels(const size_t k_maps) const;
    void publish_submap_surfel_ellipsoids(size_t k_maps, bool sliding_window = false) const;
    void publish_pose_graph() const;
    void add_axes(visualization_msgs::msg::MarkerArray& ma, const Eigen::Isometry3f& T, int64_t stamp, const std::string& ns, int& marker_id, float scale) const;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frame_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frame_surfels_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr active_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr submap_surfels_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr surfel_ellipsoids_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pose_graph_pub_;

    // Components
    std::unique_ptr<FrameProcessor> frame_processor_;
    std::unique_ptr<SurfelExtractor> surfel_extractor_;
    std::unique_ptr<ActiveMap> active_map_;

    RolloverPolicy rollover_policy_;

    // Shared state container
    std::shared_ptr<MapStateContainer> map_state_container_;

    // Pose buffer for timestamp-matched lookup
    std::deque<StampedPose> pose_buffer_;
    static constexpr size_t POSE_BUFFER_SIZE = 75; // ~200ms at 250 Hz

    // Static extrinsic: sensor-in-body
    Eigen::Isometry3f T_body_sensor_{Eigen::Isometry3f::Identity()};

    // TF broadcasting for RViz2 visualization
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    mutable int next_ellipsoid_marker_id_{0};
    // {first_id, count} for each submap currently visible in the sliding window.

    // PointCloud message offsets (cached)
    XYZOffsets xyz_off_;
    uint32_t cached_point_step_{0};
    size_t cached_field_count_{0};

    // Timing:
    StopWatch sw_;
};



} // namespace smip_uav

#endif