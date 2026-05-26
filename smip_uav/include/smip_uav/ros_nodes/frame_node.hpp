#ifndef FRAME_NODE_HPP_
#define FRAME_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <omp.h>

#include "surfel_map/frame_builder.hpp"
#include "surfel_map/frame_processor.hpp"
#include "surfel_map/frame_buffer.hpp"

#include "common/stop_watch.hpp"

namespace smip_uav {

class FrameNode : public rclcpp::Node {
public:
    struct Config {
        FrameBuilder::Config fbuild_cfg;
        FrameProcessor::Config fproc_cfg;
        FrameBuffer::Config fbuff_cfg;

        std::string map_frame;
        std::string odom_frame;
        std::string sensor_frame;
        std::string pointcloud_in_topic;
        std::string pointcloud_out_topic;

        bool is_sim{false};
        bool has_external_tf{false};
        float visualization_rate{0.0f};
    };

    explicit FrameNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());

    bool use_external_tf() const { return cfg_.has_external_tf; }

private:
    void declare_parameters();
    void load_parameters();
    bool get_transform(const rclcpp::Time& stamp);
    void pointcloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);

    // Components
    std::unique_ptr<FrameBuilder> fbuild_;
    std::unique_ptr<FrameProcessor> fproc_;
    std::unique_ptr<FrameBuffer> fbuff_;

    // ROS2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_repub_;

    // Buffers
    std::vector<PointXYZ> pts_;
    Frame frame_;
    std::vector<FrameSurfel> frame_surfels_;
    CommittedSurfels committed_surfels_;
    rclcpp::Time t_msg_;
    Eigen::Isometry3f tf_;
    
    // State
    bool is_processing_{false};
    bool is_init_{false};

    // Configuration
    Config cfg_;

    // Initial check for Cloud message field offsets
    struct XYZOffsets {
        uint32_t x{0}, y{0}, z{0};
        bool valid{false};
    };
    static inline XYZOffsets find_xyz_offsets(const sensor_msgs::msg::PointCloud2& cloud) {
        XYZOffsets off;
        bool got_x = false, got_y = false, got_z = false;

        for (const auto& f : cloud.fields) {
            // Require float32
            if (f.datatype != sensor_msgs::msg::PointField::FLOAT32 || f.count != 1) continue;

            if      (f.name == "x") { off.x = f.offset; got_x = true; }
            else if (f.name == "y") { off.y = f.offset; got_y = true; }
            else if (f.name == "z") { off.z = f.offset; got_z = true; }
        }

        off.valid = got_x && got_y && got_z;
        return off;
    }
    XYZOffsets xyz_off_;
    uint32_t cached_point_step_{0};
    size_t cached_field_count_{0};

    // Timinig
    StopWatch clock_;

};



} // namespace smip_uav

#endif