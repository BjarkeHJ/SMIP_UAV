#include "ros_nodes/voxl_tf_node.hpp"

namespace smip_uav {

VoxlTfNode::VoxlTfNode(const rclcpp::NodeOptions& opts)
: Node("voxl_tf_node", opts)
{
    this->declare_parameter("odom_frame",   "odom");
    this->declare_parameter("body_frame",   "base_link");
    this->declare_parameter("tof_frame",    "tof");
    this->declare_parameter("ground_frame", "ground");
    // this->declare_parameter("input_topic",  "/ov/odom");
    this->declare_parameter("input_topic",  "/fmu/out/vehicle_odometry");
    this->declare_parameter("px4_odometry", true);

    odom_frame_   = this->get_parameter("odom_frame").as_string();
    body_frame_   = this->get_parameter("body_frame").as_string();
    tof_frame_    = this->get_parameter("tof_frame").as_string();
    ground_frame_ = this->get_parameter("ground_frame").as_string();
    std::string topic = this->get_parameter("input_topic").as_string();
    bool px4 = this->get_parameter("px4_odometry").as_bool();

    tf_broadcaster_        = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/smip/drone_path", rclcpp::SystemDefaultsQoS());
    path_msg_.header.frame_id = odom_frame_;

    publish_static_transforms();

    if (px4) {
        RCLCPP_INFO(get_logger(), "VoxlTfNode: Listening for PX4 VehicleOdometry");
        px4_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            topic, rclcpp::QoS(10).best_effort(),
            std::bind(&VoxlTfNode::px4_odom_callback, this, std::placeholders::_1));
    } else {
        RCLCPP_INFO(get_logger(), "VoxlTfNode: Listening for VIO Odometry");
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            topic, rclcpp::SensorDataQoS(),
            std::bind(&VoxlTfNode::odom_callback, this, std::placeholders::_1));
    }

    RCLCPP_INFO(get_logger(), "VoxlTfNode: odom relay %s → TF [%s → %s]",
        topic.c_str(), odom_frame_.c_str(), body_frame_.c_str());
}

void VoxlTfNode::publish_static_transforms()
{
    // Extrinsics from voxl_calibration (FRD body convention).
    // The TF tree uses FLU (base_link), so we apply the FRD→FLU conversion:
    //   translation:  p_flu = Rx180 * p_frd  →  negate y and z
    //   rotation:     R_flu = Rx180 * R_frd * Rx180

    std::vector<geometry_msgs::msg::TransformStamped> stfs;
    auto now = this->get_clock()->now();

    // ---- #7  body → tof ------------------------------------------------
    // T_child_wrt_parent (FRD): [0.066,  0.009, -0.012]
    // R_child_to_parent  (FRD): [0,0,1 / 0,-1,0 / 1,0,0]
    // After FRD→FLU:
    //   t_flu = [0.066, -0.009, 0.012]
    //   R_flu = [[0,0,-1],[0,-1,0],[-1,0,0]]
    // +180° around tof x-axis (sensor faces +x but depth was flipped):
    //   R_final = R_flu * Rx180 = [[0,0,1],[0,1,0],[-1,0,0]] → q = [w=√½, x=0, y=√½, z=0]
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = now;
        tf.header.frame_id = body_frame_;
        tf.child_frame_id  = tof_frame_;

        tf.transform.translation.x =  0.066;
        tf.transform.translation.y = -0.009;
        tf.transform.translation.z =  0.012;

        tf.transform.rotation.w =  0.70711;
        tf.transform.rotation.x =  0.0;
        tf.transform.rotation.y =  0.70711;
        tf.transform.rotation.z =  0.0;

        stfs.push_back(tf);
    }

    // ---- #8  body → ground ---------------------------------------------
    // T_child_wrt_parent (FRD): [0.000, 0.000, 0.033]
    // R_child_to_parent  (FRD): identity
    // After FRD→FLU:
    //   t_flu = [0.0, 0.0, -0.033]   (FRD +z is down → FLU -z)
    //   R_flu = Rx180 * I * Rx180 = I → q = [w=1, 0, 0, 0]
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = now;
        tf.header.frame_id = body_frame_;
        tf.child_frame_id  = ground_frame_;

        tf.transform.translation.x =  0.0;
        tf.transform.translation.y =  0.0;
        tf.transform.translation.z = -0.033;

        tf.transform.rotation.w = 1.0;
        tf.transform.rotation.x = 0.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = 0.0;

        stfs.push_back(tf);
    }

    static_tf_broadcaster_->sendTransform(stfs);

    RCLCPP_INFO(get_logger(), "Published static transforms: [%s→%s] [%s→%s]",
        body_frame_.c_str(), tof_frame_.c_str(),
        body_frame_.c_str(), ground_frame_.c_str());
}

void VoxlTfNode::broadcast_tf(const rclcpp::Time& stamp,
    double x, double y, double z,
    double qw, double qx, double qy, double qz)
{
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id  = body_frame_;

    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = z;
    tf.transform.rotation.w = qw;
    tf.transform.rotation.x = qx;
    tf.transform.rotation.y = qy;
    tf.transform.rotation.z = qz;

    tf_broadcaster_->sendTransform(tf);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp    = stamp;
    pose.header.frame_id = odom_frame_;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation = tf.transform.rotation;

    path_msg_.header.stamp = stamp;
    path_msg_.poses.push_back(pose);
    path_pub_->publish(path_msg_);
}

void VoxlTfNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // OpenVINS world and body frames are both FRD (z-down).
    // Convert to FLU via Rx180 sandwich on quaternion, negate y and z on position.
    static const Eigen::Quaterniond q_x180(0.0, 1.0, 0.0, 0.0);
    const auto& o = msg->pose.pose.orientation;
    const Eigen::Quaterniond q_flu = (q_x180 * Eigen::Quaterniond(o.w, o.x, o.y, o.z) * q_x180).normalized();
    const auto& p = msg->pose.pose.position;

    broadcast_tf(msg->header.stamp,
        p.x, -p.y, -p.z,
        q_flu.w(), q_flu.x(), q_flu.y(), q_flu.z());
}

void VoxlTfNode::px4_odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    // PX4 VehicleOdometry: position in NED, body orientation FRD→NED.
    // Timestamp is µs from system start; convert to ROS nanoseconds.
    rclcpp::Time stamp(static_cast<int64_t>(msg->timestamp) * 1000LL);

    // NED → ENU: x=East, y=North, z=Up
    const double x =  msg->position[1];
    const double y =  msg->position[0];
    const double z = -msg->position[2];

    // FRD/NED → FLU/ENU: q_result = q_ned_to_enu * q_frd_to_ned * q_x180
    // q_ned_to_enu = 180° around (X+Y)/√2 axis = (w=0, x=√½, y=√½, z=0)
    static const Eigen::Quaterniond q_x180(0.0, 1.0, 0.0, 0.0);
    static const Eigen::Quaterniond q_ned_to_enu(0.0, M_SQRT1_2, M_SQRT1_2, 0.0);

    const Eigen::Quaterniond q_px4(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    const Eigen::Quaterniond q_enu = (q_ned_to_enu * q_px4 * q_x180).normalized();

    broadcast_tf(stamp, x, y, z,
        q_enu.w(), q_enu.x(), q_enu.y(), q_enu.z());
}

} // namespace smip_uav
