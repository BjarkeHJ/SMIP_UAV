#ifndef VIZ_UTILS_HPP_
#define VIZ_UTILS_HPP_

#include "viz_utils/viz_base.hpp"

// ==== INCLUDE SUPPORTED MESSAGE TYPES ====
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// ==== INCLUDE SUPPORTED DATA TYPES (Custom) ====
#include "common/point_types.hpp"
// #include "surfel_map/surfel.hpp"

// ==== VISUALIZATION CONVERTERS ====
namespace viz_convs {

inline sensor_msgs::msg::Image depth_mono8(
        const std::vector<float>& buf,
        uint32_t w, uint32_t h,
        const rclcpp::Time& stamp,
        const std::string& frame_id) {

    sensor_msgs::msg::Image m;
    m.header.stamp = stamp;
    m.header.frame_id = frame_id;
    m.height = h;
    m.width  = w;
    m.encoding = "mono8";
    m.is_bigendian = false;
    m.step = w;
    m.data.resize(w * h, 0);
    
    float min_d = std::numeric_limits<float>::max();
    float max_d = std::numeric_limits<float>::lowest();

    for (float v : buf) {
        if (!std::isfinite(v)) continue;
        min_d = std::min(min_d, v);
        max_d = std::max(max_d, v);
    }

    float range = max_d - min_d;
    if (range < 1e-6f) range = 1.0f;

    const float inv_range = 255.0f / range;

    for (size_t i = 0; i < buf.size(); ++i) {
        float v = std::isfinite(buf[i]) ? buf[i] : min_d;
        float norm = (v - min_d) * inv_range;
        m.data[i] = static_cast<uint8_t>(std::clamp(norm, 0.0f, 255.0f));
    }
    return m;
}

inline sensor_msgs::msg::Image normal_rgb8(
        const std::vector<float>& buf,  // nx,ny,nz interleaved
        uint32_t w, uint32_t h,
        const rclcpp::Time& stamp,
        const std::string& frame_id) {

    sensor_msgs::msg::Image m;
    m.header.stamp = stamp;
    m.header.frame_id = frame_id;
    m.height = h;
    m.width  = w;
    m.encoding = "rgb8";
    m.is_bigendian = false;
    m.step = w * 3;
    m.data.resize(w * h * 3);

    for (size_t i = 0; i < w * h; ++i) {
        m.data[3*i + 0] = static_cast<uint8_t>((buf[3*i + 0] * 0.5f + 0.5f) * 255.0f);
        m.data[3*i + 1] = static_cast<uint8_t>((buf[3*i + 1] * 0.5f + 0.5f) * 255.0f);
        m.data[3*i + 2] = static_cast<uint8_t>((buf[3*i + 2] * 0.5f + 0.5f) * 255.0f);
    }
    return m;
}

inline sensor_msgs::msg::Image weight_mono8(
        const std::vector<float>& buf,
        uint32_t w, uint32_t h,
        const rclcpp::Time& stamp,
        const std::string& frame_id) {
    
    sensor_msgs::msg::Image m;
    m.header.stamp = stamp;
    m.header.frame_id = frame_id;
    m.height = h;
    m.width = w;
    m.encoding = "mono8";
    m.is_bigendian = false;
    m.step = w;
    m.data.resize(w * h, 0);

    for (size_t i = 0; i < w * h; ++i) {
        m.data[i] = static_cast<uint8_t>(buf[i] * 255.0f);
    }
    return m;
}

inline sensor_msgs::msg::Image edge_binary(
        const std::vector<uint8_t> eh,
        const std::vector<uint8_t> ev,
        uint32_t w, uint32_t h,
        const rclcpp::Time& stamp,
        const std::string& frame_id) {
    
    sensor_msgs::msg::Image m;
    m.header.stamp = stamp;
    m.header.frame_id = frame_id;
    m.height = h;
    m.width = w;
    m.encoding = "mono8";
    m.is_bigendian = false;
    m.step = w;
    m.data.resize(w*h, 0);

    for (size_t i = 0; i < w * h; ++i) {
        if (eh[i] == 1 || ev[i] == 1) {
            m.data[i] = 255;
        }
    }
    return m;
}

inline visualization_msgs::msg::MarkerArray surfel_to_markers(
    const std::vector<FrameSurfel>& surfels,
    const rclcpp::Time& stamp,
    const std::string& frame_id,
    float scale_factor = 3.0f) {  // 3σ ellipsoid by default

    visualization_msgs::msg::MarkerArray ma;
    ma.markers.reserve(surfels.size());

    for (size_t i = 0; i < surfels.size(); ++i) {
        const auto& s = surfels[i];

        const auto c = s.centroid;
        const auto n = s.normal;
        const auto ev = s.eigenvalues;
        auto evecs = s.eigenvectors;

        if (!c.allFinite() || !ev.allFinite()) continue;

        // Ensure proper rotation (det = +1), not a reflection
        if (evecs.determinant() < 0.0f)
            evecs.col(0) = -evecs.col(0);

        Eigen::Quaternionf q(evecs);
        q.normalize();

        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = stamp;
        m.ns = "surfels";
        m.id = static_cast<int>(i);
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;

        m.pose.position.x = c.x();
        m.pose.position.y = c.y();
        m.pose.position.z = c.z();

        m.pose.orientation.x = q.x();
        m.pose.orientation.y = q.y();
        m.pose.orientation.z = q.z();
        m.pose.orientation.w = q.w();

        // Diameter = 2 * scale_factor * sqrt(eigenvalue) per axis
        // Eigenvalues sorted ascending: col(0)=smallest=normal dir
        m.scale.x = scale_factor * std::sqrt(ev(0) + 1e-6f);  // normal direction (thin)
        m.scale.y = scale_factor * std::sqrt(ev(1) + 1e-6f);  // tangent 1
        m.scale.z = scale_factor * std::sqrt(ev(2) + 1e-6f);  // tangent 2

        // Color by weight
        m.color.r = std::abs(n.x());
        m.color.g = std::abs(n.y());
        m.color.b = std::abs(n.z());
        m.color.a = 0.6f;

        // m.color.r = s.view_cos_theta;
        // m.color.g = 0.0f;
        // m.color.b = 0.0f;
        // m.color.a = 0.6f;

        m.lifetime = rclcpp::Duration::from_seconds(0.2);

        ma.markers.push_back(m);
    }

    return ma;
}

inline visualization_msgs::msg::MarkerArray map_surfels_to_markers(
    const std::vector<MapSurfel*>& surfels,
    const rclcpp::Time& stamp,
    const std::string& frame_id,
    float scale_factor = 3.0f) {

    visualization_msgs::msg::MarkerArray ma;
    ma.markers.reserve(surfels.size());

    for (size_t i = 0; i < surfels.size(); ++i) {
        const auto* s = surfels[i];
        if (!s || !s->mu.allFinite()) continue;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(s->C_shape);
        if (eig.info() != Eigen::Success) continue;
        const Eigen::Vector3f evals = eig.eigenvalues().cwiseMax(1e-8f);
        Eigen::Matrix3f evecs = eig.eigenvectors();

        if (evecs.determinant() < 0.0f) {
            evecs.col(0) = -evecs.col(0);
        }

        Eigen::Quaternionf q(evecs);
        q.normalize();

        Eigen::Vector3f n = s->normal();

        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = stamp;
        m.ns = "surfels";
        m.id = static_cast<int>(i);
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;

        m.pose.position.x = s->mu.x();
        m.pose.position.y = s->mu.y();
        m.pose.position.z = s->mu.z();

        m.pose.orientation.x = q.x();
        m.pose.orientation.y = q.y();
        m.pose.orientation.z = q.z();
        m.pose.orientation.w = q.w();

        // Eigenvalues sorted ascending: col(0)=smallest=normal dir
        m.scale.x = scale_factor * std::sqrt(evals(0) + 1e-6f);  // normal direction (thin)
        m.scale.y = scale_factor * std::sqrt(evals(1) + 1e-6f);  // tangent 1
        m.scale.z = scale_factor * std::sqrt(evals(2) + 1e-6f);  // tangent 2

        // Color by weight
        m.color.r = std::abs(n.x());
        m.color.g = std::abs(n.y());
        m.color.b = std::abs(n.z());
        m.color.a = 0.6f;

        // m.color.r = s.view_cos_theta;
        // m.color.g = 0.0f;
        // m.color.b = 0.0f;
        // m.color.a = 0.6f;

        m.lifetime = rclcpp::Duration::from_seconds(0.00);

        ma.markers.push_back(m);
    }

    return ma;
}

} // viz_convs


// ==== VISUALIZATION CHANNELS ====
namespace viz_channels {

inline VizChannel<Frame, sensor_msgs::msg::Image> frame_depth(
        Visualizer& viz, 
        const std::string& frame_id, 
        const std::string& subtopic,
        rclcpp::QoS qos
    ) {
    return viz.create<Frame, sensor_msgs::msg::Image>(subtopic, frame_id, qos,
        [](const Frame& f, const rclcpp::Time& stamp, const std::string& fid) {
            return viz_convs::depth_mono8(
                f.depth_image(),
                static_cast<uint32_t>(f.W),
                static_cast<uint32_t>(f.H),
                stamp,
                fid);
        });
}

inline VizChannel<Frame, sensor_msgs::msg::Image> frame_normal(
        Visualizer& viz,
        const std::string& frame_id,
        const std::string& subtopic,
        rclcpp::QoS qos
    ) {
    return viz.create<Frame, sensor_msgs::msg::Image>(subtopic, frame_id, qos,
        [](const Frame& f, const rclcpp::Time& stamp, const std::string& fid) {
            return viz_convs::normal_rgb8(
                f.normal_image(),
                static_cast<uint32_t>(f.W),
                static_cast<uint32_t>(f.H),
                stamp,
                fid);
        });
}

inline VizChannel<Frame, sensor_msgs::msg::Image> frame_weight(
        Visualizer& viz,
        const std::string& frame_id,
        const std::string& subtopic,
        rclcpp::QoS qos
    ) {
    return viz.create<Frame, sensor_msgs::msg::Image>(subtopic, frame_id, qos,
        [](const Frame& f, const rclcpp::Time& stamp, const std::string& fid) {
            return viz_convs::weight_mono8(
                f.weight_image(),
                static_cast<uint32_t>(f.W),
                static_cast<uint32_t>(f.H),
                stamp,
                fid);
        });
}

inline VizChannel<Frame, sensor_msgs::msg::Image> frame_edge(
        Visualizer& viz,
        const std::string& frame_id,
        const std::string& subtopic,
        rclcpp::QoS qos
    ) {
    return viz.create<Frame, sensor_msgs::msg::Image>(subtopic, frame_id, qos,
        [](const Frame& f, const rclcpp::Time& stamp, const std::string& fid) {
            return viz_convs::edge_binary(
                f.edge_h,
                f.edge_v,
                static_cast<uint32_t>(f.W),
                static_cast<uint32_t>(f.H),
                stamp,
                fid);
        });
}

inline VizChannel<std::vector<FrameSurfel>, visualization_msgs::msg::MarkerArray> surfels(
    Visualizer& viz,
    const std::string& frame_id,
    const std::string& subtopic,
    rclcpp::QoS qos
) {
    return viz.create<std::vector<FrameSurfel>, visualization_msgs::msg::MarkerArray>(subtopic, frame_id, qos,
        [](const std::vector<FrameSurfel>& S, const rclcpp::Time& stamp, const std::string& fid) {
            return viz_convs::surfel_to_markers(S, stamp, fid);
        });
}

inline VizChannel<std::vector<MapSurfel*>, visualization_msgs::msg::MarkerArray> map_surfels(
    Visualizer& viz,
    const std::string& frame_id,
    const std::string& subtopic,
    rclcpp::QoS qos
) {
    return viz.create<std::vector<MapSurfel*>, visualization_msgs::msg::MarkerArray>(subtopic, frame_id, qos,
        [](const std::vector<MapSurfel*>& S, const rclcpp::Time& stamp, const std::string& fid) {
            return viz_convs::map_surfels_to_markers(S, stamp, fid);
        });
}

} // viz_channels


#endif