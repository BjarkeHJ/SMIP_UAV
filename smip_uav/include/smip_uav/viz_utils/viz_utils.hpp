#ifndef VIZ_UTILS_HPP_
#define VIZ_UTILS_HPP_

#include "viz_utils/viz_base.hpp"

// ==== INCLUDE SUPPORTED MESSAGE TYPES ====
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// ==== INCLUDE SUPPORTED DATA TYPES (Custom) ====
#include "common/point_types.hpp"

// ==== VISUALIZATION CONVERTERS ====
namespace viz_convs {

inline sensor_msgs::msg::Image depth_mono8(
        const std::vector<float>& buf,
        uint32_t w, uint32_t h,
        float min_range, float max_range,
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

    const float inv_range = 255.0f / (max_range - min_range);
    for (size_t i = 0; i < buf.size(); ++i) {
        float clamped = std::clamp(buf[i], min_range, max_range);
        // m.data[i] = static_cast<uint8_t>((max_range - clamped) * inv_range);
        m.data[i] = static_cast<uint8_t>((clamped - min_range) * inv_range);
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
} // viz_convs


// ==== VISUALIZATION CHANNELS ====
namespace viz_channels {

inline VizChannel<Frame, sensor_msgs::msg::Image> frame_depth(
        Visualizer& viz, 
        const std::string& frame_id, 
        const std::string& subtopic,
        rclcpp::QoS qos,
        float min_range, float max_range
    ) {
    return viz.create<Frame, sensor_msgs::msg::Image>(subtopic, frame_id, qos,
        [min_range, max_range](const Frame& f, const rclcpp::Time& stamp, const std::string& fid) {
            return viz_convs::depth_mono8(
                f.depth_image(),
                static_cast<uint32_t>(f.W),
                static_cast<uint32_t>(f.H),
                min_range,
                max_range,
                stamp,
                fid);
        });
}

inline VizChannel<Frame, sensor_msgs::msg::Image> frame_normal(
        Visualizer& viz,
        const std::string& frame_id,
        const std::string& subtopic,
        rclcpp::QoS qos = rclcpp::SensorDataQoS()
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

} // viz_channels


#endif