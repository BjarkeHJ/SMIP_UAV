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

// struct SurfelVizDelta {
//     std::vector<const smip_uav::Surfel*> to_add;
//     std::vector<uint32_t> to_delete;
//     float voxel_size{0.2f};
// };

// inline visualization_msgs::msg::MarkerArray surfel_normal_rgb8(
//         const SurfelVizDelta& delta,
//         const rclcpp::Time& stamp,
//         const std::string& frame_id) {

//     visualization_msgs::msg::MarkerArray m;
//     m.markers.reserve(delta.to_add.size() + delta.to_delete.size());

//     for (uint32_t del_id : delta.to_delete) {
//         visualization_msgs::msg::Marker md;
//         md.header.stamp    = stamp;
//         md.header.frame_id = frame_id;
//         md.ns     = "surfels";
//         md.id     = static_cast<int>(del_id);
//         md.action = visualization_msgs::msg::Marker::DELETE;
//         m.markers.push_back(md);
//     }

//     for (const smip_uav::Surfel* s : delta.to_add) {
//         visualization_msgs::msg::Marker ms;
//         ms.header.stamp    = stamp;
//         ms.header.frame_id = frame_id;
//         ms.ns     = "surfels";
//         ms.id     = static_cast<int>(s->id());
//         ms.type   = visualization_msgs::msg::Marker::SPHERE;
//         ms.action = visualization_msgs::msg::Marker::ADD;

//         // --- position ---
//         const Eigen::Vector3f p = s->world_mean(delta.voxel_size);
//         ms.pose.position.x = p.x();
//         ms.pose.position.y = p.y();
//         ms.pose.position.z = p.z();

//         // --- orientation from eigenvectors ---
//         const Eigen::Matrix3f& evec = s->eigenvectors();
//         const Eigen::Vector3f& eval = s->eigenvalues();

//         Eigen::Matrix3f R;
//         R.col(0) = evec.col(2);
//         R.col(1) = evec.col(1);
//         R.col(2) = evec.col(0);

//         if (R.determinant() < 0.0f) R.col(0) = -R.col(0);

//         Eigen::Quaternionf q(R);
//         q.normalize();
//         ms.pose.orientation.x = q.x();
//         ms.pose.orientation.y = q.y();
//         ms.pose.orientation.z = q.z();
//         ms.pose.orientation.w = q.w();

//         // Evs are descending (l1 > l2 > l3)
//         ms.scale.x = 2.0f * std::sqrt(std::max(eval(2), 1e-6f));
//         ms.scale.y = 2.0f * std::sqrt(std::max(eval(1), 1e-6f));
//         ms.scale.z = 2.0f * std::sqrt(std::max(eval(0), 1e-6f));

//         // color: normal direction
//         const Eigen::Vector3f& n = s->normal();
//         ms.color.r = n.x() * 0.5f + 0.5f;
//         ms.color.g = n.y() * 0.5f + 0.5f;
//         ms.color.b = n.z() * 0.5f + 0.5f;
//         ms.color.a = 0.85f;

//         m.markers.push_back(ms);
//     }

//     return m;
// }


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

// inline VizChannel<viz_convs::SurfelVizDelta, visualization_msgs::msg::MarkerArray> surfels_normal(
//         Visualizer& viz,
//         const std::string& frame_id,
//         const std::string& subtopic,
//         rclcpp::QoS qos
//     ) {
//     return viz.create<viz_convs::SurfelVizDelta, visualization_msgs::msg::MarkerArray>(subtopic, frame_id, qos,
//         [](const viz_convs::SurfelVizDelta& delta, const rclcpp::Time& stamp, const std::string& fid) {
//             return viz_convs::surfel_normal_rgb8(delta, stamp, fid);
//         });
// }

} // viz_channels


#endif