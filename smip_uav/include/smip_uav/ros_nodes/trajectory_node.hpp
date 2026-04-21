#ifndef TRAJECTORY_NODE_HPP_
#define TRAJECTORY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "smip_uav/action/flight_path.hpp"

using FlightPath = smip_uav::action::FlightPath;
using GoalHandle = rclcpp_action::ServerGoalHandle<FlightPath>;

class TrajectoryNode : public rclcpp::Node {
public:
    TrajectoryNode();

private:
    rclcpp_action::Server<FlightPath>::SharedPtr server_;
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const FlightPath::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        std::shared_ptr<GoalHandle> gh);
    
    void handle_accepted(std::shared_ptr<GoalHandle> gh);
    void execute_tick();

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
    rclcpp::TimerBase::SharedPtr tick_timer_;

    // goal state
    std::shared_ptr<GoalHandle> active_gh_;
    struct Waypoint {
        Eigen::Vector3f pos;
        float yaw;
    };
    std::vector<Waypoint> waypoints_;
    uint32_t seg_idx_{0};
    float pos_tol_{0.05f};
    float yaw_tol_{0.01f};
    float v_max_{1.0f};

    // interpolation
    Eigen::Vector3f interp_pos_{Eigen::Vector3f::Zero()};
    float interp_yaw_{0.0f};

    // helpers
    static float wrap_pi(float a);
    static float shortest_yaw_diff(float from, float to);
    static geometry_msgs::msg::Quaternion yaw2quat(float yaw);
    static float quat2yaw(const geometry_msgs::msg::Quaternion& q);
    geometry_msgs::msg::PoseStamped make_target_msg();

    // params
    static constexpr int tick_ms_{50};
    static constexpr double k_dt_{tick_ms_ / 1000.0};
    std::string frame_id_{"odom"};
};

#endif