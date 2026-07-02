#include "ros_nodes/global_map_node.hpp"
#include <geometry_msgs/msg/point.hpp>

#include <cmath>
#include <unordered_map>
#include <Eigen/Eigenvalues>

#include "core/surfel.hpp"

namespace smip_uav {

GlobalMapNode::GlobalMapNode(std::shared_ptr<MapStateContainer> container) : Node("global_map_node"), map_state_container_(container) {
    ba_ = std::make_unique<BundleAdjustment>();

    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(opt_period_s_),
        std::bind(&GlobalMapNode::opt_cycle, this),
        cb_group_
    );    

    graph_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/smip_uav/global_pose_graph", rclcpp::QoS(1));
    surfel_point_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/smip_uav/global_surfel_points", rclcpp::QoS(1));
    surfel_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/smip_uav/global_surfel_markers", rclcpp::QoS(1));

    RCLCPP_INFO(this->get_logger(), "GlobalMapNode ready!");
}

void GlobalMapNode::opt_cycle() {
    // Hand off the latest map<-odom correction to the front-end, which broadcasts it
    // stamped with the IMU clock (single source of timing truth for the tf tree).
    map_state_container_->write_map_odom(T_map_odom_);

    const MapSnapshot snap = map_state_container_->snapshot();
    if (snap.total_submaps < 2) return;

    bool any_new = false;
    for (const auto& v : snap.views) {
        if (ba_->has(v.id)) continue;
        map_state_container_->read_submap(v.id, [&](const FrozenSubmap& fs) {
            ba_->ingest_submap(v.id, fs);
        });
        any_new = true;
    }

    if (!any_new && snap.total_submaps == last_submap_count_) return;

    last_submap_count_ = snap.total_submaps;

    const BundleAdjustment::Result res = ba_->optimize(snap);
    if (res.corrections.empty()) return;

    // Derive T_map_odom from the latest submap: corrected pose vs its odometric origin
    const PoseCorrection& latest = res.corrections.back();
    for (const auto& v : snap.views) {
        if (v.id == latest.id) {
            T_map_odom_ = latest.T_submap_world_corrected * v.T_submap_world.inverse();
            break;
        }
    }

    map_state_container_->apply_pose_corrections(res.corrections);

    const auto& st = res.stats;
    RCLCPP_INFO(this->get_logger(),
        "[BA] submaps=%u clusters=%u (mean %.1f contribs) "
        "dropped[few=%u plan=%u w=%u] cost %.3g -> %.3g "
        "iters=%d outer=%d | assoc %.1f ms solve %.1f ms",
        snap.total_submaps, st.clusters, st.mean_contribs,
        st.dropped_few_submaps, st.dropped_planarity, st.dropped_weight,
        st.cost_initial, st.cost_final, st.iterations, st.outer_loops,
        st.t_associate_ms, st.t_solve_ms 
    );

    publish_pose_graph();
    publish_surfel_points();
    publish_surfel_markers();
}

void GlobalMapNode::publish_pose_graph() {
    const MapSnapshot snap = map_state_container_->snapshot();
    if (snap.views.empty()) return;

    visualization_msgs::msg::MarkerArray ma;
    const rclcpp::Time stamp(snap.views.back().stamp_ns_end);

    visualization_msgs::msg::Marker nodes;
    nodes.header.frame_id = map_frame_;
    nodes.header.stamp = stamp;
    nodes.ns = "ba_nodes";
    nodes.id = 0;
    nodes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    nodes.action = visualization_msgs::msg::Marker::ADD;
    nodes.pose.orientation.w = 1.0;
    nodes.scale.x = nodes.scale.y = nodes.scale.z = 0.12;
    nodes.color.r = 0.2f; 
    nodes.color.g = 0.8f;
    nodes.color.b = 0.3f;
    nodes.color.a = 1.0f;

    visualization_msgs::msg::Marker chain;
    chain.header = nodes.header;
    chain.ns = "ba_chain";
    chain.id = 1;
    chain.type = visualization_msgs::msg::Marker::LINE_STRIP;
    chain.action = visualization_msgs::msg::Marker::ADD;
    chain.pose.orientation.w = 1.0;
    chain.scale.x = 0.02;
    chain.color.r = 0.9f;
    chain.color.g = 0.9f;
    chain.color.b = 0.2f;
    chain.color.a = 0.8f;

    for (const auto& v : snap.views) {
        geometry_msgs::msg::Point p;
        p.x = v.T_submap_world.translation().x();
        p.y = v.T_submap_world.translation().y();
        p.z = v.T_submap_world.translation().z();
        nodes.points.push_back(p);
        chain.points.push_back(p);
    }

    ma.markers.push_back(nodes);
    ma.markers.push_back(chain);
    graph_pub_->publish(ma);
}

void GlobalMapNode::publish_surfel_points() {
    const MapSnapshot snap = map_state_container_->snapshot();
    if (snap.views.empty()) return;

    uint32_t total = 0;
    for (const auto& v : snap.views) total += v.surfel_count;
    if (total == 0) return;

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = rclcpp::Time(snap.views.back().stamp_ns_end);
    msg.height = 1;
    msg.width = total;
    msg.is_dense = false;
    msg.is_bigendian = false;

    sensor_msgs::msg::PointField fx, fy, fz, frgb;
    fx.name = "x";
    fx.offset = 0;
    fx.datatype = sensor_msgs::msg::PointField::FLOAT32;
    fx.count = 1;
    fy.name = "y";
    fy.offset = 4;
    fy.datatype = sensor_msgs::msg::PointField::FLOAT32;
    fy.count = 1;
    fz.name = "z";
    fz.offset = 8;
    fz.datatype = sensor_msgs::msg::PointField::FLOAT32;
    fz.count = 1;
    frgb.name = "rgb";
    frgb.offset = 12;
    frgb.datatype = sensor_msgs::msg::PointField::FLOAT32;
    frgb.count = 1;

    msg.fields = {fx, fy, fz, frgb};
    msg.point_step = 16;
    msg.row_step = msg.point_step * total;
    msg.data.resize(msg.row_step);

    auto make_rgb = [](float r, float g, float b) -> float {
        uint32_t u = (uint32_t(r * 255.0f) << 16) |
                     (uint32_t(g * 255.0f) << 8)  |
                     (uint32_t(b * 255.0f));
        float f;
        std::memcpy(&f, &u, 4);
        return f;
    };

    const bool by_conf = false;
    uint8_t* ptr = msg.data.data();
    uint8_t* const end = ptr + msg.data.size();
    for (const auto& view : snap.views) {
        map_state_container_->read_submap(view.id, [&](const FrozenSubmap& fs) {
            const Eigen::Matrix3f R = fs.T_submap_world.linear();
            for (const Surfel& s : fs.surfels) {
                if (ptr + 16 > end) return;
                const Eigen::Vector3f p = fs.T_submap_world * s.position;
                float x = p.x();
                float y = p.y();
                float z = p.z();
                float rgb;
                if (by_conf) {
                    const float c = std::clamp(s.confidence, 0.0f, 1.0f);
                    rgb = make_rgb(1.0f - c, c, 0.0f);
                }
                else {
                    const Eigen::Vector3f n = R * s.normal;
                    rgb = make_rgb(
                        (n.x() + 1.0f) * 0.5f,
                        (n.y() + 1.0f) * 0.5f,
                        (n.z() + 1.0f) * 0.5f
                    );
                }

                std::memcpy(ptr + 0, &x, 4);
                std::memcpy(ptr + 4, &y, 4);
                std::memcpy(ptr + 8, &z, 4);
                std::memcpy(ptr + 12, &rgb, 4);
                ptr += 16;
            }
        });
    }

    const size_t written = static_cast<size_t>(ptr - msg.data.data());
    if (written != msg.data.size()) {
        msg.width = static_cast<uint32_t>(written / msg.point_step);
        msg.row_step = static_cast<uint32_t>(written);
        msg.data.resize(written);
    }

    surfel_point_pub_->publish(msg);
}

void GlobalMapNode::publish_surfel_markers() {
    const MapSnapshot snap = map_state_container_->snapshot();
    if (snap.views.empty()) return;

    const rclcpp::Time stamp(snap.views.back().stamp_ns_end);
    visualization_msgs::msg::MarkerArray ma;

    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    del.ns = "global_surfel_ellipsoids";
    ma.markers.push_back(del);

    const bool by_conf = false;
    int id_counter = 0;
    for (const auto& view : snap.views) {
        map_state_container_->read_submap(view.id, [&](const FrozenSubmap& fs) {
            const Eigen::Matrix3f R = fs.T_submap_world.linear();
            for (const Surfel& s : fs.surfels) {
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(s.shape);
                if (eig.info() != Eigen::Success) continue;
                const Eigen::Vector3f evals = eig.eigenvalues().cwiseMax(0.0f);

                Eigen::Matrix3f evecs_world = R * eig.eigenvectors();
                if (evecs_world.determinant() < 0.0f) evecs_world.col(0) = -evecs_world.col(0);
                
                const Eigen::Quaternionf q(evecs_world);
                const Eigen::Vector3f p = fs.T_submap_world * s.position;

                visualization_msgs::msg::Marker m;
                m.header.frame_id = map_frame_;
                m.header.stamp = stamp;
                m.ns = "global_surfel_ellipsoids";
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
                m.scale.y = 2.0 * std::sqrt(evals(1));
                m.scale.z = 2.0 * std::sqrt(evals(2));

                if (by_conf) {
                    const float c = std::clamp(s.confidence, 0.0f, 1.0f);
                    m.color.r = 1.0f - c;
                    m.color.g = c;
                    m.color.b = 0.0f;
                }
                else {
                    const Eigen::Vector3f n = R * s.normal;
                    m.color.r = (n.x() + 1.0f) * 0.5f;
                    m.color.g = (n.y() + 1.0f) * 0.5f;
                    m.color.b = (n.z() + 1.0f) * 0.5f;
                }
                m.color.a = 0.8f;
                ma.markers.push_back(m);
            }
        });
    }

    surfel_marker_pub_->publish(ma);
}

} // namespace smip_uav