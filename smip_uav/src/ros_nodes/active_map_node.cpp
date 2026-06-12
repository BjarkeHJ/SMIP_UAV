#include "ros_nodes/active_map_node.hpp"
#include "utils/frame_transform.hpp"

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

ActiveMapNode::ActiveMapNode(std::shared_ptr<MapStateContainer> container) : Node("active_map_node"), map_state_container_(container) {
    omp_set_num_threads(4);

    // Component Initialization
    frame_processor_ = std::make_unique<FrameProcessor>();
    surfel_extractor_ = std::make_unique<SurfelExtractor>();

    // Callback group
    cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );

    // Subscription options
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = cb_group_;

    // RO2 Sub/Pub
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "tof_pc",
        rclcpp::SensorDataQoS(),
        std::bind(&ActiveMapNode::pointcloud_callback, this, std::placeholders::_1),
        sub_opt
    );

    pose_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "fmu/out/vehicle_odometry",
        rclcpp::SensorDataQoS(),
        std::bind(&ActiveMapNode::pose_callback, this, std::placeholders::_1),
        sub_opt
    );

    // TF broadcasters
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Visualization
    frame_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/smip_uav/frame_raw_points",
        rclcpp::SensorDataQoS()
    );
    frame_surfels_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/smip_uav/frame_surfel_cloud",
        rclcpp::SensorDataQoS()
    );
    submap_surfels_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/smip_uav/frozen",
        10
    );
    surfel_ellipsoids_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/smip_uav/surfel_ellipsoids",
        10
    );
    pose_graph_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/smip_uav/pose_graph",
        10
    );

    // Publish static body-tof
    T_body_sensor_.setIdentity();
    T_body_sensor_.rotate(Eigen::Quaternionf(0.70711f, 0.0f, 0.70711f, 0.0f));
    T_body_sensor_.pretranslate(Eigen::Vector3f(0.066f, -0.009f, 0.012f));

    auto stf = tf2::eigenToTransform(T_body_sensor_.cast<double>());
    stf.header.stamp    = this->get_clock()->now();
    stf.header.frame_id = "base_link";
    stf.child_frame_id  = "sensor_frame";
    static_tf_broadcaster_->sendTransform(stf);

    RCLCPP_INFO(this->get_logger(), "ActiveMapNode ready!");
}

// void ActiveMapNode::pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {
void ActiveMapNode::pose_callback(px4_msgs::msg::VehicleOdometry::SharedPtr pose_msg) {
    // PX4 publishes position in NED and orientation in FRD — convert to ENU/FLU
    const Eigen::Vector3f ned_pos(
        // static_cast<float>(pose_msg->pose.position.x),
        // static_cast<float>(pose_msg->pose.position.y),
        // static_cast<float>(pose_msg->pose.position.z)
        static_cast<float>(pose_msg->position[0]),
        static_cast<float>(pose_msg->position[1]),
        static_cast<float>(pose_msg->position[2])
    );
    const Eigen::Quaternionf frd_q(
        // static_cast<float>(pose_msg->pose.orientation.w),
        // static_cast<float>(pose_msg->pose.orientation.x),
        // static_cast<float>(pose_msg->pose.orientation.y),
        // static_cast<float>(pose_msg->pose.orientation.z)
        static_cast<float>(pose_msg->q[0]),
        static_cast<float>(pose_msg->q[1]),
        static_cast<float>(pose_msg->q[2]),
        static_cast<float>(pose_msg->q[3])
    );

    // T_world_body: PX4 NED/FRD → ENU/FLU (extrinsic applied lazily in get_current_pose)
    StampedPose sp;
    sp.T_world.setIdentity();
    sp.T_world.rotate(frame_transform::TF_BODY_FRD_FLU(frd_q));
    sp.T_world.pretranslate(frame_transform::TF_WORLD_NED_ENU(ned_pos));
    sp.stamp_ns  = static_cast<int64_t>(pose_msg->timestamp) * 1000LL;
    sp.cov       = Eigen::Matrix<float,6,6>::Zero();
    pose_buffer_.push_back(sp);
    if (pose_buffer_.size() > POSE_BUFFER_SIZE) pose_buffer_.pop_front();

    // Broadcast odom-base_link transform
    const Eigen::Vector3f    t = sp.T_world.translation();
    const Eigen::Quaternionf q(sp.T_world.rotation());
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp            = rclcpp::Time(sp.stamp_ns);
    tf_msg.header.frame_id         = "odom";
    tf_msg.child_frame_id          = "base_link";
    tf_msg.transform.translation.x = t.x();
    tf_msg.transform.translation.y = t.y();
    tf_msg.transform.translation.z = t.z();
    tf_msg.transform.rotation.w    = q.w();
    tf_msg.transform.rotation.x    = q.x();
    tf_msg.transform.rotation.y    = q.y();
    tf_msg.transform.rotation.z    = q.z();
    tf_broadcaster_->sendTransform(tf_msg);
}

void ActiveMapNode::pointcloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    // Repub pointcloud?

    sw_.tic();
    const rclcpp::Time t_msg = cloud_msg->header.stamp;
    const int64_t t_ns = static_cast<int64_t>(t_msg.nanoseconds());

    std::optional<StampedPose> pose_opt = get_current_pose(t_ns);
    if (!pose_opt.has_value()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "No pose available - dropping frame..."
        );
        return;
    }
    const StampedPose& pose = *pose_opt;

    // Construct and process Frame
    // std::unique_ptr<Frame> frame = std::make_unique<Frame>(240, 180, pose.T_world, t_ns); // should just be the StampedPose?
    std::unique_ptr<Frame> frame = std::make_unique<Frame>(240, 180, pose);
    convert_pointcloud_message(cloud_msg, *frame);
    frame_processor_->process(*frame);
    surfel_extractor_->extract(*frame);


    if (frame->surfels.empty()) return;
    
    // Initialize active map on first frame
    if (!active_map_) {
        active_map_ = std::make_unique<ActiveMap>(pose.T_world, t_ns);
        RCLCPP_INFO(this->get_logger(), "Started first ActiveMap");
    }

    // Register Frame into ActiveMap
    active_map_->register_frame(*frame, pose);

    // Evaluate rollover
    const RolloverSignal sig = rollover_policy_.evaluate(pose, *active_map_, *map_state_container_);
    if (sig.action != RolloverAction::CONTINUE) {
        handle_rollover(pose, sig, t_ns);
    }

    publish_frame_points(*frame);
    publish_frame(*frame);

    double t = sw_.toc();
    RCLCPP_INFO(this->get_logger(), "Surfels in Frame: %zu. Computation Time: %f.3 ms", frame->surfels.size(), t);
}

void ActiveMapNode::handle_rollover(const StampedPose& pose, const RolloverSignal& signal, int64_t stamp_ns) {
    sw_.tic();
    FrozenSubmap frozen = active_map_->freeze(stamp_ns);
    double t_freeze = sw_.toc();
    const SubmapId id = map_state_container_->commit_submap(std::move(frozen));

    publish_pose_graph();
    publish_submap_surfel_ellipsoids(1, false);

    size_t n_surfels = 0;
    map_state_container_->read_submap(id, [&](const FrozenSubmap& fs) {
        n_surfels = fs.surfels.size();
    });
    RCLCPP_INFO(this->get_logger(),
        "Committed submap %u (%zu surfels, %.1f m, %u frames) -- Freeze Time: %f.3 ms",
        id,
        n_surfels,
        active_map_->accumulated_translation(),
        active_map_->frame_count(),
        t_freeze
    );

    if (signal.action == RolloverAction::LOOP_CLOSURE_HINT) {
        RCLCPP_INFO(this->get_logger(),
            "Loop-closure hint: new submap %u overlaps frozen submap %u",
            id, *signal.overlap_id
        );
    }

    active_map_ = std::make_unique<ActiveMap>(pose.T_world, stamp_ns);
}

std::optional<StampedPose> ActiveMapNode::get_current_pose(int64_t scan_stamp) const {
    
    if (pose_buffer_.empty()) return std::nullopt;

    auto best = std::min_element(pose_buffer_.begin(), pose_buffer_.end(),
        [scan_stamp](const StampedPose& a, const StampedPose& b) {
            return std::abs(a.stamp_ns - scan_stamp) < std::abs(b.stamp_ns - scan_stamp);
        });
    StampedPose sp = *best;

    sp.T_world = sp.T_world * T_body_sensor_;  // tf from body to sensor frame
    return sp;
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

void ActiveMapNode::publish_frame_points(const Frame& frame) const {
    const std::vector<PointNormal>& pts = frame.pixels.pointnormals;
    const std::vector<uint8_t>& valid = frame.pixels.validities;
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = rclcpp::Time(frame.meta.pose.stamp_ns);
    msg.header.frame_id = "sensor_frame";
    msg.height = 1;
    msg.width = static_cast<uint32_t>(pts.size());
    msg.is_dense = false;
    msg.is_bigendian = false;

    sensor_msgs::msg::PointField fx, fy, fz, frgb;
    fx.name = "x"; fx.offset = 0; fx.datatype = sensor_msgs::msg::PointField::FLOAT32; fx.count = 1;
    fy.name = "y"; fy.offset = 4; fy.datatype = sensor_msgs::msg::PointField::FLOAT32; fy.count = 1;
    fz.name = "z"; fz.offset = 8; fz.datatype = sensor_msgs::msg::PointField::FLOAT32; fz.count = 1;
    frgb.name = "rgb"; frgb.offset = 12; frgb.datatype = sensor_msgs::msg::PointField::FLOAT32; frgb.count = 1;
    msg.fields = {fx, fy, fz, frgb};
    msg.point_step = 16;
    msg.row_step = msg.point_step * pts.size();
    msg.data.resize(msg.row_step);

    auto make_rgb = [](float r, float g, float b) -> float {
        r = r * 0.5f + 0.5f;
        g = g * 0.5f + 0.5f;
        b = b * 0.5f + 0.5f;
        uint32_t u = (uint32_t(r * 255.0f) << 16)
                | (uint32_t(g * 255.0f) <<  8)
                |  uint32_t(b * 255.0f);
        float f;
        std::memcpy(&f, &u, 4);
        return f;
    };

    uint8_t* ptr = msg.data.data();
    for (size_t i = 0; i < pts.size(); ++i) {
        const Eigen::Vector3f& p = pts[i].p;
        const Eigen::Vector3f& n = pts[i].n;
        const Eigen::Vector3f& n_w = frame.meta.pose.T_world.rotation().transpose() * n;
        float x = p.x();
        float y = p.y();
        float z = p.z();
        float rgb;
        if (!valid[i]) {
            rgb = make_rgb(0.0f, 0.0f, 0.0f);
        }
        else {
            rgb = make_rgb(n_w.x(), n_w.y(), n_w.z());
        }
        std::memcpy(ptr + 0, &x, 4);
        std::memcpy(ptr + 4, &y, 4);
        std::memcpy(ptr + 8, &z, 4);
        std::memcpy(ptr + 12, &rgb, 4);
        ptr += 16;
    }

    frame_points_pub_->publish(msg);
}

void ActiveMapNode::publish_frame(const Frame& frame) const {
    const auto& surfels = frame.surfels;
    sensor_msgs::msg::PointCloud2 s_cloud;
    s_cloud.header.stamp = rclcpp::Time(frame.meta.pose.stamp_ns);
    s_cloud.header.frame_id = "sensor_frame";
    s_cloud.height = 1;
    s_cloud.width = static_cast<uint32_t>(surfels.size());
    s_cloud.is_dense = false;
    s_cloud.is_bigendian = false;

    // Field layout: x(0) y(4) z(8) rgb(12)  →  point_step = 16
    auto push_field = [&](const std::string& name, uint32_t offset, uint8_t dtype) {
        sensor_msgs::msg::PointField f;
        f.name = name; f.offset = offset; f.datatype = dtype; f.count = 1;
        s_cloud.fields.push_back(f);
    };
    push_field("x",   0,  sensor_msgs::msg::PointField::FLOAT32);
    push_field("y",   4,  sensor_msgs::msg::PointField::FLOAT32);
    push_field("z",   8,  sensor_msgs::msg::PointField::FLOAT32);
    push_field("rgb", 12, sensor_msgs::msg::PointField::FLOAT32);
    s_cloud.point_step = 16;
    s_cloud.row_step = s_cloud.point_step * s_cloud.width;
    s_cloud.data.resize(s_cloud.row_step);

    auto make_rgb = [](float r,float g, float b) -> float {
        uint32_t u = (uint32_t(r*255.0f) << 16) | (uint32_t(g*255.0f) << 8) | uint32_t(b*255.0f);
        float f; 
        std::memcpy(&f, &u, 4);
        return f;
    };

    uint8_t* ptr = s_cloud.data.data();
    for (const Surfel& s : surfels) {
        const Eigen::Vector3f& n = s.normal;
        if (n.norm() < 1e-3f) continue;
        float x = s.position.x();
        float y = s.position.y();
        float z = s.position.z();
        float rgb = make_rgb(1.0f - s.confidence, s.confidence, 0.0f);
        std::memcpy(ptr + 0, &x, 4);
        std::memcpy(ptr + 4, &y, 4);
        std::memcpy(ptr + 8, &z, 4);
        std::memcpy(ptr + 12, &rgb, 4);
        ptr += 16;
    }

    frame_surfels_pub_->publish(s_cloud);
}

void ActiveMapNode::publish_submap_surfels(const size_t k_maps) const {
    const MapSnapshot snap = map_state_container_->snapshot();
    if (snap.views.empty()) return;

    const size_t start = snap.views.size() > k_maps ? snap.views.size() - k_maps : 0;

    uint32_t total = 0;
    for (size_t i = start; i < snap.views.size(); ++i) {
        total += snap.views[i].surfel_count;
    }
    if (total == 0) return;

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = "odom";
    msg.header.stamp = rclcpp::Time(snap.views.back().stamp_ns_end);
    msg.height = 1;
    msg.width = total;
    msg.is_dense = false;
    msg.is_bigendian = false;

    sensor_msgs::msg::PointField fx, fy, fz, frgb;
    fx.name = "x"; fx.offset = 0; fx.datatype = sensor_msgs::msg::PointField::FLOAT32; fx.count = 1;
    fy.name = "y"; fy.offset = 4; fy.datatype = sensor_msgs::msg::PointField::FLOAT32; fy.count = 1;
    fz.name = "z"; fz.offset = 8; fz.datatype = sensor_msgs::msg::PointField::FLOAT32; fz.count = 1;
    frgb.name = "rgb"; frgb.offset = 12; frgb.datatype = sensor_msgs::msg::PointField::FLOAT32; frgb.count = 1;
    msg.fields = {fx, fy, fz, frgb};
    msg.point_step = 16;
    msg.row_step = msg.point_step * total;
    msg.data.resize(msg.row_step);

    auto make_rgb = [](float r,float g, float b) -> float {
        uint32_t u = (uint32_t(r*255.0f) << 16) | (uint32_t(g*255.0f) << 8) | uint32_t(b*255.0f);
        float f;
        std::memcpy(&f, &u, 4);
        return f;
    };

    uint8_t* ptr = msg.data.data();
    for (size_t i = start; i < snap.views.size(); ++i) {
        const auto& view = snap.views[i];
        map_state_container_->read_submap(view.id, [&](const FrozenSubmap& fs) {
            const Eigen::Matrix3f R = fs.T_submap_world.linear();
            for (const Surfel& s : fs.surfels) {
                const Eigen::Vector3f p = fs.T_submap_world * s.position;
                const Eigen::Vector3f n_world = R * s.normal;
                float x = p.x();
                float y = p.y();
                float z = p.z();
                const float rgb = make_rgb(
                    (n_world.x() + 1.0f) * 0.5f,
                    (n_world.y() + 1.0f) * 0.5f,
                    (n_world.z() + 1.0f) * 0.5f);
                std::memcpy(ptr + 0, &x, 4);
                std::memcpy(ptr + 4, &y, 4);
                std::memcpy(ptr + 8, &z, 4);
                std::memcpy(ptr + 12, &rgb, 4);
                ptr += 16;
            }
        });
    }

    submap_surfels_pub_->publish(msg);
}

void ActiveMapNode::publish_submap_surfel_ellipsoids(const size_t k_maps, bool sliding_window) const {
    const MapSnapshot snap = map_state_container_->snapshot();
    if (snap.views.empty()) return;

    const size_t start = snap.views.size() > k_maps ? snap.views.size() - k_maps : 0;
    const rclcpp::Time stamp = rclcpp::Time(snap.views.back().stamp_ns_end);

    visualization_msgs::msg::MarkerArray ma;

    auto make_conf_rgb = [](float c) -> float {
        return std::clamp(c, 0.0f, 1.0f);
    };

    auto add_surfel_markers = [&](const FrozenSubmap& fs, int& id_counter) {
        const Eigen::Matrix3f R_world = fs.T_submap_world.linear();
        for (const Surfel& s : fs.surfels) {
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(s.shape);
            if (eig.info() != Eigen::Success) continue;

            const Eigen::Vector3f evals = eig.eigenvalues().cwiseMax(0.0f);

            Eigen::Matrix3f evecs_world = R_world * eig.eigenvectors();
            if (evecs_world.determinant() < 0.0f) evecs_world.col(0) = -evecs_world.col(0);

            const Eigen::Quaternionf q(evecs_world);
            const Eigen::Vector3f p = fs.T_submap_world * s.position;
            const Eigen::Vector3f n_world = R_world * s.normal;

            visualization_msgs::msg::Marker m;
            m.header.frame_id = "odom";
            m.header.stamp = stamp;
            m.ns = "surfel_ellipsoids";
            m.id = id_counter++;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;

            m.pose.position.x = p.x();
            m.pose.position.y = p.y();
            m.pose.position.z = p.z();
            m.pose.orientation.w = q.w();
            m.pose.orientation.x = q.x();
            m.pose.orientation.y = q.y();
            m.pose.orientation.z = q.z();

            constexpr float kMinThickness = 0.01f;
            m.scale.x = std::max(2.0f * std::sqrt(evals(0)), kMinThickness);
            m.scale.y = 2.0f * std::sqrt(evals(1));
            m.scale.z = 2.0f * std::sqrt(evals(2));

            const float conf = make_conf_rgb(s.confidence);
            m.color.r = 1.0f - conf;
            m.color.g = conf;
            m.color.b = 0.0f;
            m.color.a = 0.8f;

            ma.markers.push_back(m);
        }
    };

    if (sliding_window) {
        // DELETEALL then republish the k_maps most recent submaps with fresh ids.
        // Message stays small since k_maps is typically small.
        visualization_msgs::msg::Marker del;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        del.ns = "surfel_ellipsoids";
        ma.markers.push_back(del);

        int local_id = 0;
        for (size_t i = start; i < snap.views.size(); ++i) {
            map_state_container_->read_submap(snap.views[i].id, [&](const FrozenSubmap& fs) {
                add_surfel_markers(fs, local_id);
            });
        }
    } else {
        // Persistent mode: only publish the newest submap, accumulate in RViz.
        map_state_container_->read_submap(snap.views.back().id, [&](const FrozenSubmap& fs) {
            add_surfel_markers(fs, next_ellipsoid_marker_id_);
        });
    }

    surfel_ellipsoids_pub_->publish(ma);
}

void ActiveMapNode::publish_pose_graph() const {
    const MapSnapshot snap = map_state_container_->snapshot();
    if (snap.views.empty()) return;

    visualization_msgs::msg::MarkerArray ma;
    int marker_id = 0;

    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    del.ns = "graph_poses";
    ma.markers.push_back(del);

    visualization_msgs::msg::Marker traj;
    traj.header.frame_id = "odom";
    traj.ns = "graph_trajectory";
    traj.id = marker_id++;
    traj.type = visualization_msgs::msg::Marker::LINE_STRIP;
    traj.action = visualization_msgs::msg::Marker::ADD;
    traj.scale.x = 0.02f;
    traj.color.r = 0.7f;
    traj.color.g = 0.7f;
    traj.color.b = 0.7f;
    traj.color.a = 1.0f;
    traj.lifetime = rclcpp::Duration(0,0); //infinite

    for (const auto& view : snap.views) {
        traj.header.stamp = rclcpp::Time(view.stamp_ns_start);
        const Eigen::Vector3f t = view.T_submap_world.translation();
        geometry_msgs::msg::Point pt;
        pt.x = t.x();
        pt.y = t.y();
        pt.z = t.z();
        traj.points.push_back(pt);

        add_axes(ma, view.T_submap_world, view.stamp_ns_start, "graph_poses", marker_id, 0.15f);

        visualization_msgs::msg::Marker txt;
        txt.header.frame_id = "odom";
        txt.header.stamp = rclcpp::Time(view.stamp_ns_start);
        txt.ns = "submap_labels";
        txt.id = marker_id++;
        txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        txt.action = visualization_msgs::msg::Marker::ADD;
        txt.pose.position.x = t.x();
        txt.pose.position.y = t.y();
        txt.pose.position.z = t.z() + 0.15f;
        txt.scale.z = 0.08f;
        txt.color.r = 1.0f;
        txt.color.g = 1.0f;
        txt.color.b = 1.0f;
        txt.color.a = 1.0f;
        txt.lifetime = rclcpp::Duration(0,0);
        txt.text = "S" + std::to_string(view.id) + "\n(" + std::to_string(view.surfel_count) + ")";
        ma.markers.push_back(txt);
    }

    ma.markers.push_back(traj);
    pose_graph_pub_->publish(ma);
}

void ActiveMapNode::add_axes(visualization_msgs::msg::MarkerArray& ma, const Eigen::Isometry3f& T, int64_t stamp, const std::string& ns, int& marker_id, float scale) const {
    const Eigen::Vector3f origin = T.translation();
    const Eigen::Matrix3f R = T.rotation();

    const std::array<Eigen::Vector3f, 3> axes = {
        R.col(0), R.col(1), R.col(2)
    };
    const std::array<std::array<float,3>, 3> colors = {{
        {1,0,0},{0,1,0},{0,0,1}
    }};
    
    for (int a = 0; a < 3; ++a) {
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = "odom";
        arrow.header.stamp = rclcpp::Time(stamp);
        arrow.ns = ns;
        arrow.id = marker_id++;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.lifetime = rclcpp::Duration(0,0);

        geometry_msgs::msg::Point p_start, p_end;
        p_start.x = origin.x();
        p_start.y = origin.y();
        p_start.z = origin.z();
        const Eigen::Vector3f tip = origin + scale * axes[a];
        p_end.x = tip.x();
        p_end.y = tip.y();
        p_end.z = tip.z();

        arrow.points.push_back(p_start);
        arrow.points.push_back(p_end);
        arrow.scale.x = scale * 0.08f;
        arrow.scale.y = scale * 0.15f;
        arrow.color.r = colors[a][0];
        arrow.color.g = colors[a][1];
        arrow.color.b = colors[a][2];
        arrow.color.a = 1.0f;

        ma.markers.push_back(arrow);
    }
}

} // namespace smip_uav