#include "ros_nodes/map_node.hpp"

namespace smip_uav {

SurfelMapNode::SurfelMapNode(const rclcpp::NodeOptions& options) : Node("surfel_map_node", options) {
    // Set number of threds for parallelization
    omp_set_num_threads(4);

    // Declare and load parameters
    declare_parameters();
    load_parameters();

    // Visualizer
    viz_ = std::make_unique<Visualizer>(this, "/smip");
    depth_ch_ = viz_channels::frame_depth(*viz_, cfg_.sensor_tof_frame, "tof_depth", rclcpp::SensorDataQoS()); // TODO: ranges from sensor config
    normal_ch_ = viz_channels::frame_normal(*viz_, cfg_.sensor_tof_frame, "tof_normal", rclcpp::SensorDataQoS());
    weight_ch_ = viz_channels::frame_weight(*viz_, cfg_.sensor_tof_frame, "tof_weight", rclcpp::SensorDataQoS());
    edge_ch_ = viz_channels::frame_edge(*viz_, cfg_.sensor_tof_frame, "tof_edge", rclcpp::SensorDataQoS());
    surfel_ch_ = viz_channels::surfels(*viz_, cfg_.sensor_tof_frame, "tof_surfel", rclcpp::SensorDataQoS());
    superpixel_ch_ = viz_channels::frame_superpixels(*viz_, cfg_.sensor_tof_frame, "tof_superpixels", rclcpp::SensorDataQoS());
    map_ch_ = viz_channels::map_surfels_delta(*viz_, cfg_.odom_frame, "map_surfel", rclcpp::SensorDataQoS());

    // Components
    fbuild_ = std::make_unique<FrameBuilder>(cfg_.fbuild_cfg);
    fproc_ = std::make_unique<FrameProcessor>(cfg_.fproc_cfg);
    smap_ = std::make_unique<SurfelMap>(cfg_.smap_cfg);

    // ROS2 TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // ROS2 Subscription
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cfg_.pointcloud_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&SurfelMapNode::pointcloud_data_callback, this, std::placeholders::_1)
    );

    // Publish timer
    if (cfg_.visualization_rate > 0.0) {
        pub_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / cfg_.visualization_rate),
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
    // ROS Node configuration
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("sensor_tof_frame", "tof");
    this->declare_parameter("pointcloud_topic", "/tof_pc");
    this->declare_parameter("simulation", false);
    this->declare_parameter("tf", false);
    this->declare_parameter("visualization_rate", 0.0);
    cfg_.odom_frame = this->get_parameter("odom_frame").as_string();
    cfg_.sensor_tof_frame = this->get_parameter("sensor_tof_frame").as_string();
    cfg_.pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();
    cfg_.is_sim = this->get_parameter("simulation").as_bool();
    cfg_.has_external_tf = this->get_parameter("tf").as_bool();
    cfg_.visualization_rate = this->get_parameter("visualization_rate").as_double();

    // FrameBuilder::Config
    this->declare_parameter("builder.tof_res_x",            (int)180);
    this->declare_parameter("builder.tof_res_y",            (int)240);
    this->declare_parameter("builder.min_range",            0.3);
    this->declare_parameter("builder.max_range",            10.0);
    this->declare_parameter("builder.pixel_pitch",          0.01071);
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

    // SurfelMap::Config
    this->declare_parameter("map.pi_spawn",                 0.005);
    this->declare_parameter("map.spawn_residual",           0.75);
    this->declare_parameter("map.gamma_forget",             0.99);
    this->declare_parameter("map.normal_sigma",             M_PI / 8.0);
    this->declare_parameter("map.merge_normal_k",           0.5);
    this->declare_parameter("map.merge_mahal_sq",           3.0);
    this->declare_parameter("map.merge_interval",           (int)5);
}

void SurfelMapNode::load_parameters() {
    auto& b = cfg_.fbuild_cfg;
    b.tof_res_x = (size_t)this->get_parameter("builder.tof_res_x").as_int();
    b.tof_res_y = (size_t)this->get_parameter("builder.tof_res_y").as_int();
    b.min_range = (float)this->get_parameter("builder.min_range").as_double();
    b.max_range = (float)this->get_parameter("builder.max_range").as_double();
    b.pixel_pitch = (float)this->get_parameter("builder.pixel_pitch").as_double();
    b.transpose_input = !cfg_.is_sim;
    b.enable_ground_filter = this->get_parameter("builder.enable_ground_filter").as_bool();
    b.ground_z_min = (float)this->get_parameter("builder.ground_z_min").as_double();
    b.ds_factor = (int)this->get_parameter("builder.ds_factor").as_int();
    b.edge_normal_th = (float)this->get_parameter("builder.edge_normal_th").as_double();
    b.edge_depth_min = (float)this->get_parameter("builder.edge_depth_min").as_double();

    auto& p = cfg_.fproc_cfg;
    p.seed_spacing = (size_t)this->get_parameter("processor.seed_spacing").as_int();
    p.perturb_window = (size_t)this->get_parameter("processor.perturb_window").as_int();
    p.min_px = (size_t)this->get_parameter("processor.min_px").as_int();
    p.w_spatial = (float)this->get_parameter("processor.w_spatial").as_double();
    p.w_normal = (float)this->get_parameter("processor.w_normal").as_double();
    p.max_cluster_dist = (float)this->get_parameter("processor.max_cluster_dist").as_double();
    p.pixel_pitch = (float)this->get_parameter("processor.pixel_pitch").as_double();

    auto& s = cfg_.smap_cfg;
    s.pi_spawn = (float)this->get_parameter("map.pi_spawn").as_double();
    s.spawn_residual = (float)this->get_parameter("map.spawn_residual").as_double();
    s.gamma_forget = (float)this->get_parameter("map.gamma_forget").as_double();
    s.normal_sigma = (float)this->get_parameter("map.normal_sigma").as_double();
    s.merge_normal_k = (float)this->get_parameter("map.merge_normal_k").as_double();
    s.merge_mahal_sq = (float)this->get_parameter("map.merge_mahal_sq").as_double();
    s.merge_interval = (uint32_t)this->get_parameter("map.merge_interval").as_int();

    auto& g = s.grid_config;
    g.voxel_size = (float)this->get_parameter("grid.voxel_size").as_double();
    g.initial_bucket_count = (size_t)this->get_parameter("grid.initial_bucket_count").as_int();
    g.max_load_factor = (float)this->get_parameter("grid.max_load_factor").as_double();
}

bool SurfelMapNode::get_transform(const rclcpp::Time& stamp) {
    try {
        auto transform = tf_buffer_->lookupTransform(cfg_.odom_frame, cfg_.sensor_tof_frame, stamp, rclcpp::Duration::from_nanoseconds(20'000'000));
        // auto transform = tf_buffer_->lookupTransform(cfg_.odom_frame, cfg_.sensor_tof_frame, tf2::TimePointZero);
        tf_ = tf2::transformToEigen(transform.transform).cast<float>();
        return true;
    }
    catch (const tf2::TransformException& ex) {
        // RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        return false;
    }
}

void SurfelMapNode::pointcloud_data_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {    
    // Get current transform
    if (!get_transform(cloud_msg->header.stamp)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Failed to get transform from %s to %s",
            cloud_msg->header.frame_id.c_str(), cfg_.odom_frame.c_str()
        );
        return;
    }
    
    // capture timestamp of message
    t_msg_ = cloud_msg->header.stamp;
    int64_t timestamp_ns = static_cast<int64_t>(t_msg_.nanoseconds());

    // Check for cloud_msg field offsets (first only - or if something changes - should not)
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
    for (size_t i = 0; i < n; ++i) {
        const uint8_t* p = data + i * step;
        float x, y, z;
        std::memcpy(&x, p + xyz_off_.x, 4);
        std::memcpy(&y, p + xyz_off_.y, 4);
        std::memcpy(&z, p + xyz_off_.z, 4);
        pts_.push_back({x, y, z});
    }

    process(timestamp_ns);
}

void SurfelMapNode::process(int64_t timestamp_ns) {
    clock_.tic();

     // Build frame from scan points
    GroundPlane gnd;
    gnd.normal_z = tf_.rotation().transpose() * Eigen::Vector3f::UnitZ();
    gnd.offset_z = -gnd.normal_z.dot(tf_.inverse().translation());
    current_frame_ = fbuild_->process(pts_, timestamp_ns, &gnd);

    // Process the frame (Local surfel extraction)
    current_frame_surfels_ = fproc_->process(current_frame_);

    // Update buffer containing recent local surfels
    // TODO

    // Update surfel map
    // TODO: UPDATE SO THE OUTPUT FROM BUFFER ENTERS HERE
    smap_->update_map(current_frame_surfels_, tf_, timestamp_ns);

    const double t_update = clock_.toc();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "SurfelMap Update Time (total): %f - Surfels in Frame: %ld - Map Size: %ld", t_update, current_frame_surfels_.size(), smap_->surfel_count());
}

void SurfelMapNode::publish_map() {
    // Publish visualization
    if (cfg_.is_sim) {
        depth_ch_.publish(current_frame_, this->get_clock()->now());
        normal_ch_.publish(current_frame_, this->get_clock()->now());
        weight_ch_.publish(current_frame_, this->get_clock()->now());
        edge_ch_.publish(current_frame_, this->get_clock()->now());
        superpixel_ch_.publish(SuperpixelImage{fproc_->labels(),
            static_cast<uint32_t>(current_frame_.W),
            static_cast<uint32_t>(current_frame_.H)}, this->get_clock()->now());
    }

    surfel_ch_.publish(current_frame_surfels_, t_msg_);
    auto deleted_snapshot = smap_->deleted_ids();
    map_ch_.publish(MapSurfelDelta{smap_->get_updated_surfels(), std::move(deleted_snapshot)}, this->get_clock()->now());
}


} //smip_uav
