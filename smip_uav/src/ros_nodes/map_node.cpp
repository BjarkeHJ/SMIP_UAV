#include "ros_nodes/map_node.hpp"

namespace smip_uav {

SurfelMapNode::SurfelMapNode(const rclcpp::NodeOptions& options) : Node("surfel_map_node", options) {
    // Declare parameters
    declare_parameters();

    // Visualizer
    viz_ = std::make_unique<Visualizer>(this, "/smip");
    depth_ch_ = viz_channels::frame_depth(*viz_, tof_frame_, "tof_depth", rclcpp::SensorDataQoS(), 0.1f, 10.0f); // TODO: ranges from sensor config
    normal_ch_ = viz_channels::frame_normal(*viz_, tof_frame_, "tof_normal", rclcpp::SensorDataQoS());
    weight_ch_ = viz_channels::frame_weight(*viz_, tof_frame_, "tof_weight", rclcpp::SensorDataQoS());

    // Preprocessing
    preproc_ = std::make_unique<SensorDataPreprocess>(SensorDataPreprocess::Config{});
    
    // ROS2 TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // ROS2 Subscription
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&SurfelMapNode::pointcloud_data_callback, this, std::placeholders::_1)
    );

    pts_.reserve(240 * 180); // TODO: from sensor config (W x H)

    RCLCPP_INFO(this->get_logger(), "SurfelMap node initialized");
}

void SurfelMapNode::declare_parameters() {
    this->declare_parameter("global_frame", "odom");
    this->declare_parameter("sensor_tof_frame", "lidar_frame");
    // this->declare_parameter("pointcloud_topic", "/x500/lidar_front/points_raw");
    this->declare_parameter("pointcloud_topic", "/tof_pc");
    this->declare_parameter("simulation", true);

    global_frame_ = this->get_parameter("global_frame").as_string();
    tof_frame_ = this->get_parameter("sensor_tof_frame").as_string();
    pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
    is_sim = this->get_parameter("simulation").as_bool();
}

bool SurfelMapNode::get_transform(const rclcpp::Time& ts) {
    try {
        auto transform = tf_buffer_->lookupTransform(global_frame_, tof_frame_, ts, rclcpp::Duration::from_seconds(0.1));
        tf_ = tf2::transformToEigen(transform.transform).cast<float>();
        return true;
    }
    catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        return false;
    }
}

void SurfelMapNode::pointcloud_data_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    // Get current transform
    if (!get_transform(cloud_msg->header.stamp)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Failed to get transform from %s to %s",
            cloud_msg->header.frame_id.c_str(), global_frame_.c_str()
        );
        // return;
    }

    // extract point data
    pts_.clear();
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (is_sim) {
            pts_.push_back({*iter_x, *iter_y, *iter_z});
        }
        else {
            // transform incoming points from optical frame to FLU (x = z_opt, y = -x_opt, z = -y_opt)
            pts_.push_back({*iter_z, -*iter_x, -*iter_y});
        }
    }

    // run preprocess
    GroundPlane gnd;
    gnd.normal_z = tf_.rotation().transpose() * Eigen::Vector3f::UnitZ();
    gnd.offset_z = -gnd.normal_z.dot(tf_.inverse().translation());
    current_frame_ = preproc_->process(pts_, &gnd);

    // Publish visualization
    depth_ch_.publish(current_frame_, this->get_clock()->now());
    normal_ch_.publish(current_frame_, this->get_clock()->now());
    weight_ch_.publish(current_frame_, this->get_clock()->now());
}




} //smip_uav
