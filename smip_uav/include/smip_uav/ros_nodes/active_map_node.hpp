#ifndef SMIP_ACTIVE_MAP_NODE_HPP_
#define SMIP_ACTIVE_MAP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <omp.h>

#include "core/frame.hpp"
#include "mapping/frame_processor.hpp"
#include "mapping/surfel_extractor.hpp"

namespace smip_uav {

struct XYZOffsets {
    uint32_t x{0}, y{0}, z{0};
    bool valid{false};
};

class ActiveMapNode : public rclcpp::Node {
public:
    
    explicit ActiveMapNode();

private:
    void pointcloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    void convert_pointcloud_message(sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg, Frame& frame);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

    // Components
    std::unique_ptr<FrameProcessor> frame_processor_;
    std::unique_ptr<SurfelExtractor> surfel_extractor_;

    // PointCloud message offsets (cached)
    XYZOffsets xyz_off_;
    uint32_t cached_point_step_{0};
    size_t cached_field_count_{0};
};



} // namespace smip_uav

#endif