#ifndef VIZ_UTILS_HPP_
#define VIZ_UTILS_HPP_

#include "viz_utils/viz_base.hpp"

// ==== INCLUDE SUPPORTED MESSAGE TYPES ====
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// ==== INCLUDE SUPPORTED DATA TYPES (Custom) ====
#include "common/point_types.hpp"
#include "surfel_map/surfel.hpp"

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

inline visualization_msgs::msg::MarkerArray surfel_normal_rgb8(
        std::vector<const smip_uav::Surfel*> buf,
        const rclcpp::Time& stamp,
        const std::string& frame_id) {

    // Marker array
    visualization_msgs::msg::MarkerArray m;

    // Delete previous
    visualization_msgs::msg::Marker md;
    md.header.stamp = stamp;
    md.header.frame_id = frame_id;
    md.ns = "surfels";
    md.action = visualization_msgs::msg::Marker::DELETEALL;
    m.markers.push_back(md);

    // Update
    visualization_msgs::msg::Marker ms;
    ms.header.stamp = stamp;
    ms.header.frame_id = frame_id;
    ms.ns = "surfels";
    ms.type = visualization_msgs::msg::Marker::CYLINDER;
    ms.action = visualization_msgs::msg::Marker::ADD;

    int marker_id = 0;
    for (size_t i = 0; i < buf.size(); ++i) {
        const smip_uav::Surfel* s = buf[i];
        ms.id = marker_id++;

        // pos
        const Eigen::Vector3f& p = s->world_mean(0.3f); // HARDCODED!!!
        ms.pose.position.x = p.x();
        ms.pose.position.y = p.y();
        ms.pose.position.z = p.z();

        // ori
        const Eigen::Matrix3f& c = s->covariance();
        const Eigen::Vector3f& ev1 = c.col(0);
        const Eigen::Vector3f& ev2 = c.col(1);
        Eigen::Matrix3f R;
        R.col(0) = ev1;
        R.col(1) = ev2;
        R.col(2) = s->normal();
        if (R.determinant() < 0) R.col(1) = -R.col(1);

        Eigen::Quaternionf q(R);
        q.normalize();

        ms.pose.orientation.x = q.x();
        ms.pose.orientation.y = q.y();
        ms.pose.orientation.z = q.z();
        ms.pose.orientation.w = q.w();

        // scale
        const Eigen::Vector3f& evals = s->eigenvalues();
        ms.scale.x = 2.0f * std::sqrt(std::max(evals(0), 1e-6f));
        ms.scale.y = 2.0f * std::sqrt(std::max(evals(1), 1e-6f));
        ms.scale.z = 0.005f;

        // color
        ms.color.r = 1.0f;
        ms.color.g = 0.0f;
        ms.color.b = 0.0f;
        ms.color.a = 0.8f;

        m.markers.push_back(ms);
    }

    return m;
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

inline VizChannel<std::vector<const smip_uav::Surfel*>, visualization_msgs::msg::MarkerArray> surfels_red(
        Visualizer& viz,
        const std::string& frame_id,
        const std::string& subtopic,
        rclcpp::QoS qos
    ) {
    return viz.create<std::vector<const smip_uav::Surfel*>, visualization_msgs::msg::MarkerArray>(subtopic, frame_id, qos,
        [](const std::vector<const smip_uav::Surfel*>& s, const rclcpp::Time& stamp, const std::string& fid) {
            return viz_convs::surfel_normal_rgb8(
                s,
                stamp,
                fid
            );
        });
}

} // viz_channels


#endif