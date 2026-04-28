#ifndef MAP_NODE_HPP_
#define MAP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <unordered_set>
#include <omp.h>

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
    SurfelMap::Config load_parameters();
    void pointcloud_data_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    bool get_transform();

    // std::unique_ptr<FrameBuilder> frame_builder_;
    // std::unique_ptr<FrameProcessor> frame_processor_;
    std::unique_ptr<SurfelMap> smap_;

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

    // Visualization
    std::unique_ptr<Visualizer> viz_;
    VizChannel<Frame, sensor_msgs::msg::Image> depth_ch_;
    VizChannel<Frame, sensor_msgs::msg::Image> normal_ch_;
    VizChannel<Frame, sensor_msgs::msg::Image> weight_ch_;
    VizChannel<Frame, sensor_msgs::msg::Image> edge_ch_;
    VizChannel<std::vector<FrameSurfel>, visualization_msgs::msg::MarkerArray> surfel_ch_;
    VizChannel<MapSurfelDelta, visualization_msgs::msg::MarkerArray> map_ch_;
    VizChannel<SuperpixelImage, sensor_msgs::msg::Image> superpixel_ch_;

    // Timing
    StopWatch clock_;

    bool is_sim{true};
};

} // smip_uav


#endif