#include "ros_nodes/active_map_node.hpp"

namespace smip_uav {

namespace {
XYZOffsets find_xyz_offsets(const sensor_msgs::msg::PointCloud2& cloud) {
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
} // anonymous static namespace

ActiveMapNode::ActiveMapNode() : Node("active_map_node") {
    omp_set_num_threads(4);

    // Component Initialization
    frame_processor_ = std::make_unique<FrameProcessor>();

    // RO2 Sub/Pub
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "tof_pc",
        rclcpp::SensorDataQoS(),
        std::bind(&ActiveMapNode::pointcloud_callback, this, std::placeholders::_1)
    );

}

void ActiveMapNode::pointcloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    // Repub pointcloud?

    // Get transform
    Eigen::Isometry3f T_sensor_world = Eigen::Isometry3f::Identity();

    // Construct Frame
    const rclcpp::Time t_msg = cloud_msg->header.stamp;
    const int64_t t_ns = static_cast<int64_t>(t_msg.nanoseconds());

    std::unique_ptr<Frame> frame = std::make_unique<Frame>(240, 180, T_sensor_world, t_ns);
    convert_pointcloud_message(cloud_msg, *frame);

    // Fill/Process Frame
    frame_processor_->process(*frame);

    // Extract Surfels from Frame
    
    // Relay Frame ownership to ActiveSubMap


    // 

}

void ActiveMapNode::convert_pointcloud_message(sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg, Frame& frame) {
    // Check cloud_msg field offsets (first msg only if structure is static)
    if (!xyz_off_.valid || cloud_msg->point_step != cached_point_step_ || cloud_msg->fields.size() != cached_field_count_) {
        xyz_off_ = find_xyz_offsets(*cloud_msg);
        cached_point_step_ = cloud_msg->point_step;
        cached_field_count_ = cloud_msg->fields.size();
        if (!xyz_off_.valid) {
            RCLCPP_ERROR(get_logger(), "PointCloud2 missing FLOAT32 x/y/z fields");
            return;
        }
    }

    std::vector<PointNormal>& pns = frame.pixels.pointnormals;

    const uint8_t* data = cloud_msg->data.data();
    const uint32_t step = cloud_msg->point_step;
    const size_t n = cloud_msg->width * cloud_msg->height;
    
    for (size_t i = 0; i < n; ++i) {
        const uint8_t* p = data + i * step;
        float x, y, z;
        std::memcpy(&x, p + xyz_off_.x, 4);
        std::memcpy(&y, p + xyz_off_.y, 4);
        std::memcpy(&z, p + xyz_off_.z, 4);
        pns[i].p = {x,y,z};
    }
}

} // namespace smip_uav