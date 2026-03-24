#ifndef MAP_NODE_HPP_
#define MAP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "surfel_map/sensor_data_preprocess.hpp"
#include "surfel_map/surfel_map.hpp"

#include "common/stop_watch.hpp"
#include "viz_utils/viz_utils.hpp"

namespace smip_uav {

class SurfelMapNode : public rclcpp::Node {
public:
    explicit SurfelMapNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());

    // Some cross-node interface for getting the map or segment of the map

private:
    void declare_parameters();
    void pointcloud_data_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    bool get_transform(const rclcpp::Time& ts);

    std::unique_ptr<SensorDataPreprocess> preproc_;

    // ROS2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

    // Frames, topics, etc
    std::string global_frame_;
    std::string tof_frame_;
    std::string pointcloud_topic_;
    bool do_viz_{true};
    double viz_rate_{0.0};

    // Buffers/Variables
    std::vector<PointXYZ> pts_;
    Eigen::Isometry3f tf_;
    Frame current_frame_;
    
    // Visualization
    std::unique_ptr<Visualizer> viz_;
    VizChannel<Frame, sensor_msgs::msg::Image> depth_ch_;
    VizChannel<Frame, sensor_msgs::msg::Image> normal_ch_;

    // Timing
    StopWatch clock_;
};

} // smip_uav


#endif