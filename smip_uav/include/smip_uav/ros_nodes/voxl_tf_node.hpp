#ifndef VOXL_TF_NODE_HPP_
#define VOXL_TF_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <Eigen/Geometry>
#include <px4_msgs/msg/vehicle_odometry.hpp>

namespace smip_uav {

// Relays OpenVINS odometry (FRD body frame) to TF as a FLU body frame,
// and publishes static VOXL2 sensor extrinsics (body→tof, body→ground).
// Extrinsics are given in FRD convention and converted to FLU on publication.
class VoxlTfNode : public rclcpp::Node {
public:
    explicit VoxlTfNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void px4_odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void broadcast_tf(const rclcpp::Time& stamp, double x, double y, double z,
                      double qw, double qx, double qy, double qz);
    void publish_static_transforms();

    std::string odom_frame_;
    std::string body_frame_;
    std::string tof_frame_;
    std::string ground_frame_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_;
};

} // namespace smip_uav

#endif
