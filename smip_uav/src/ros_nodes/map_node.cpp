#include "ros_nodes/map_node.hpp"

namespace smip_uav {

SurfelMapNode::SurfelMapNode(const rclcpp::NodeOptions& options) : Node("surfel_map_node", options) {
    // Set number of threds for parallelization
    omp_set_num_threads(4);

    // Declare parameters
    declare_parameters();

    // Visualizer
    viz_ = std::make_unique<Visualizer>(this, "/smip");
    depth_ch_ = viz_channels::frame_depth(*viz_, tof_frame_, "tof_depth", rclcpp::SensorDataQoS()); // TODO: ranges from sensor config
    normal_ch_ = viz_channels::frame_normal(*viz_, tof_frame_, "tof_normal", rclcpp::SensorDataQoS());
    weight_ch_ = viz_channels::frame_weight(*viz_, tof_frame_, "tof_weight", rclcpp::SensorDataQoS());
    edge_ch_ = viz_channels::frame_edge(*viz_, tof_frame_, "tof_edge", rclcpp::SensorDataQoS());
    surfel_ch_ = viz_channels::surfels(*viz_, tof_frame_, "tof_surfel", rclcpp::SensorDataQoS());
    superpixel_ch_ = viz_channels::frame_superpixels(*viz_, tof_frame_, "tof_superpixels", rclcpp::SensorDataQoS());
    map_ch_ = viz_channels::map_surfels_delta(*viz_, global_frame_, "map_surfel", rclcpp::SensorDataQoS());

    // SurfelMap
    smap_ = std::make_unique<SurfelMap>(load_parameters());

    // ROS2 TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // ROS2 Subscription
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&SurfelMapNode::pointcloud_data_callback, this, std::placeholders::_1)
    );

    // Publish timer
    if (viz_rate_ > 0.0) {
        pub_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / viz_rate_),
            std::bind(&SurfelMapNode::publish_map, this)
        );
    }

    pts_.reserve(
        (size_t)this->get_parameter("builder.tof_res_x").as_int() *
        (size_t)this->get_parameter("builder.tof_res_y").as_int()
    );

    RCLCPP_INFO(this->get_logger(), "SurfelMap node initialized");
}
    
void SurfelMapNode::declare_parameters() {
    // ROS / frame configuration
    this->declare_parameter("global_frame", "odom");
    this->declare_parameter("sensor_tof_frame", "tof");
    this->declare_parameter("pointcloud_topic", "/tof_pc");
    this->declare_parameter("simulation", false);
    this->declare_parameter("visualization_rate", 0.0);

    global_frame_     = this->get_parameter("global_frame").as_string();
    tof_frame_        = this->get_parameter("sensor_tof_frame").as_string();
    pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
    is_sim            = this->get_parameter("simulation").as_bool();
    viz_rate_         = this->get_parameter("visualization_rate").as_double();

    // FrameBuilder::Config
    this->declare_parameter("builder.tof_res_x",            (int)180);
    this->declare_parameter("builder.tof_res_y",            (int)240);
    this->declare_parameter("builder.min_range",            0.3);
    this->declare_parameter("builder.max_range",            10.0);
    this->declare_parameter("builder.pixel_pitch",          0.01071);
    this->declare_parameter("builder.transpose_input",      true);
    this->declare_parameter("builder.enable_ground_filter", false);
    this->declare_parameter("builder.ground_z_min",         0.01);
    this->declare_parameter("builder.ds_factor",            (int)1);
    this->declare_parameter("builder.edge_normal_th",       0.52);
    this->declare_parameter("builder.edge_depth_min",       0.02);

    // FrameProcessor::Config
    this->declare_parameter("processor.seed_spacing",       (int)4);
    this->declare_parameter("processor.perturb_window",     (int)1);
    this->declare_parameter("processor.min_px",             (int)12);
    this->declare_parameter("processor.w_spatial",          1.0);
    this->declare_parameter("processor.w_normal",           1.0);
    this->declare_parameter("processor.max_cluster_dist",   1.0);
    this->declare_parameter("processor.pixel_pitch",        0.01071);

    // VoxelGrid::Config
    this->declare_parameter("grid.voxel_size",              0.25);
    this->declare_parameter("grid.initial_bucket_count",    (int)10000);
    this->declare_parameter("grid.max_load_factor",         0.75);

    // SurfelMap::Config (GMM / integration params)
    this->declare_parameter("map.pi_spawn",                 0.005);
    this->declare_parameter("map.spawn_residual",           0.75);
    this->declare_parameter("map.gamma_forget",             0.99);
    this->declare_parameter("map.normal_sigma",             M_PI / 8.0);
    this->declare_parameter("map.merge_normal_k",           0.5);
    this->declare_parameter("map.merge_mahal_sq",           3.0);
    this->declare_parameter("map.merge_interval",           (int)5);
}

SurfelMap::Config SurfelMapNode::load_parameters() {
    SurfelMap::Config cfg;

    auto& b = cfg.builder_config;
    b.tof_res_x            = (size_t)this->get_parameter("builder.tof_res_x").as_int();
    b.tof_res_y            = (size_t)this->get_parameter("builder.tof_res_y").as_int();
    b.min_range            = (float)this->get_parameter("builder.min_range").as_double();
    b.max_range            = (float)this->get_parameter("builder.max_range").as_double();
    b.pixel_pitch          = (float)this->get_parameter("builder.pixel_pitch").as_double();
    b.transpose_input      = this->get_parameter("builder.transpose_input").as_bool();
    b.enable_ground_filter = this->get_parameter("builder.enable_ground_filter").as_bool();
    b.ground_z_min         = (float)this->get_parameter("builder.ground_z_min").as_double();
    b.ds_factor            = (int)this->get_parameter("builder.ds_factor").as_int();
    b.edge_normal_th       = (float)this->get_parameter("builder.edge_normal_th").as_double();
    b.edge_depth_min       = (float)this->get_parameter("builder.edge_depth_min").as_double();

    auto& p = cfg.processor_config;
    p.seed_spacing         = (size_t)this->get_parameter("processor.seed_spacing").as_int();
    p.perturb_window       = (size_t)this->get_parameter("processor.perturb_window").as_int();
    p.min_px               = (size_t)this->get_parameter("processor.min_px").as_int();
    p.w_spatial            = (float)this->get_parameter("processor.w_spatial").as_double();
    p.w_normal             = (float)this->get_parameter("processor.w_normal").as_double();
    p.max_cluster_dist     = (float)this->get_parameter("processor.max_cluster_dist").as_double();
    p.pixel_pitch          = (float)this->get_parameter("processor.pixel_pitch").as_double();

    auto& g = cfg.grid_config;
    g.voxel_size           = (float)this->get_parameter("grid.voxel_size").as_double();
    g.initial_bucket_count = (size_t)this->get_parameter("grid.initial_bucket_count").as_int();
    g.max_load_factor      = (float)this->get_parameter("grid.max_load_factor").as_double();

    cfg.pi_spawn           = (float)this->get_parameter("map.pi_spawn").as_double();
    cfg.spawn_residual     = (float)this->get_parameter("map.spawn_residual").as_double();
    cfg.gamma_forget       = (float)this->get_parameter("map.gamma_forget").as_double();
    cfg.normal_sigma       = (float)this->get_parameter("map.normal_sigma").as_double();
    cfg.merge_normal_k     = (float)this->get_parameter("map.merge_normal_k").as_double();
    cfg.merge_mahal_sq     = (float)this->get_parameter("map.merge_mahal_sq").as_double();
    cfg.merge_interval     = (uint32_t)this->get_parameter("map.merge_interval").as_int();

    return cfg;
}

bool SurfelMapNode::get_transform() {
    try {
        auto transform = tf_buffer_->lookupTransform(global_frame_, tof_frame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(0));        
        tf_ = tf2::transformToEigen(transform.transform).cast<float>();
        return true;
    }
    catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        return false;
    }
}

void SurfelMapNode::pointcloud_data_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    clock_.tic();
    
    // Get current transform
    if (!get_transform()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Failed to get transform from %s to %s",
            cloud_msg->header.frame_id.c_str(), global_frame_.c_str()
        );
        return;
    }
    
    t_msg_ = cloud_msg->header.stamp;
    int64_t timestamp_ns = static_cast<int64_t>(t_msg_.nanoseconds());

    // Check for cloud_msg field offsets (first only - or if something changes)
    if (!xyz_off_.valid || cloud_msg->point_step  != cached_point_step_ || cloud_msg->fields.size() != cached_field_count_) {
        xyz_off_ = find_xyz_offsets(*cloud_msg);
        cached_point_step_   = cloud_msg->point_step;
        cached_field_count_  = cloud_msg->fields.size();
        if (!xyz_off_.valid) {
            RCLCPP_ERROR(get_logger(), "PointCloud2 missing FLOAT32 x/y/z fields");
            return;
        }
    }

    // extract point data
    pts_.clear();
    const uint8_t* data = cloud_msg->data.data();
    const uint32_t step = cloud_msg->point_step;
    const size_t n = cloud_msg->width * cloud_msg->height;
    if (is_sim) {
        for (size_t i = 0; i < n; ++i) {
            const uint8_t* p = data + i * step;
            float x, y, z;
            std::memcpy(&x, p + xyz_off_.x, 4);
            std::memcpy(&y, p + xyz_off_.y, 4);
            std::memcpy(&z, p + xyz_off_.z, 4);
            pts_.push_back({x, y, z});
        }
    } else {
        for (size_t i = 0; i < n; ++i) {
            const uint8_t* p = data + i * step;
            float x, y, z;
            std::memcpy(&x, p + xyz_off_.x, 4);
            std::memcpy(&y, p + xyz_off_.y, 4);
            std::memcpy(&z, p + xyz_off_.z, 4);
            // pts_.push_back({z, -x, -y});
            pts_.push_back({x, y, z});
        }
    }

    // Update SurfelMap with points...
    current_frame_surfels_.clear();
    smap_->update(pts_, tf_, timestamp_ns, &current_frame_surfels_);
    current_frame_ = smap_->frame();

    const double t_update = clock_.toc();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "SurfelMap Update Time (total): %f - Surfels in Frame: %ld - Map Size: %ld", t_update, current_frame_surfels_.size(), smap_->surfel_count());
}

void SurfelMapNode::publish_map() {
    // Publish visualization
    // depth_ch_.publish(current_frame_, this->get_clock()->now());
    // normal_ch_.publish(current_frame_, this->get_clock()->now());
    // weight_ch_.publish(current_frame_, this->get_clock()->now());
    // edge_ch_.publish(current_frame_, this->get_clock()->now());
    // superpixel_ch_.publish(SuperpixelImage{smap_->frame_labels(),
    //     static_cast<uint32_t>(current_frame_.W),
    //     static_cast<uint32_t>(current_frame_.H)}, t_msg);

    surfel_ch_.publish(current_frame_surfels_, t_msg_);
    auto deleted_snapshot = smap_->deleted_ids();
    map_ch_.publish(MapSurfelDelta{smap_->get_updated_surfels(), std::move(deleted_snapshot)}, this->get_clock()->now());
}


} //smip_uav
