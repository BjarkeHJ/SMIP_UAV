#ifndef VIZ_BASE_HPP_
#define VIZ_BASE_HPP_

#include <rclcpp/rclcpp.hpp>

// VizChannel: Genericly typed channel for converting data to ros2 message type
template <typename T, typename MsgT>
class VizChannel {
public:
    // Converter: (data, stamp, frame_id) -> ROS2 msg (MsgT)
    using Converter = std::function<MsgT(const T&, const rclcpp::Time&, const std::string&)>;
    
    VizChannel() = default;
    VizChannel(typename rclcpp::Publisher<MsgT>::SharedPtr pub, 
               Converter conv,
               std::string frame_id)
        : pub_(std::move(pub)), conv_(std::move(conv)), frame_id_(std::move(frame_id)) {} 
    
    void publish(const T& data, const rclcpp::Time& stamp) const {
        if (pub_) pub_->publish(conv_(data, stamp, frame_id_));
    }
    
    explicit operator bool() const {return pub_ != nullptr; }
    const std::string frame_id() const { return frame_id_; }

private:
    typename rclcpp::Publisher<MsgT>::SharedPtr pub_;
    Converter conv_;
    std::string frame_id_;
};

class Visualizer {
public:
    explicit Visualizer(rclcpp::Node* node, 
                     std::string topic_ns = "/smip")
        : node_(node), ns_(std::move(topic_ns)) {
            RCLCPP_INFO(node_->get_logger(), "Visualizer Initialized");
        }

    template <typename T, typename MsgT>
    VizChannel<T, MsgT> create(
        const std::string& subtopic, 
        const std::string& frame_id,
        const rclcpp::QoS& qos,
        typename VizChannel<T, MsgT>::Converter converter
    ) {
        auto pub = node_->create_publisher<MsgT>(ns_ + "/" + subtopic, qos);
        return VizChannel<T, MsgT>(std::move(pub), std::move(converter), frame_id);
    }

private:
    rclcpp::Node* node_;
    std::string ns_;
};




#endif