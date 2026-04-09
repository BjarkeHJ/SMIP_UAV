#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/path.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "common/stop_watch.hpp"
#include "smip_uav/action/flight_path.hpp"

using FlightPath = smip_uav::action::FlightPath;

class TestFlight : public rclcpp::Node {
public:
    TestFlight() : Node("test_flight_node") {
        client_ = rclcpp_action::create_client<FlightPath>(this, "flight_path");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_tick_ms_), std::bind(&TestFlight::send_once, this));
        RCLCPP_INFO(this->get_logger(), "Beginning Test FLight!");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<FlightPath>::SharedPtr client_;

    std::string frame_id_{"odom"};
    int timer_tick_ms_{50};
    bool sent_{false};
    void send_once();

    geometry_msgs::msg::Quaternion yaw2quat(float yaw) {
        geometry_msgs::msg::Quaternion q;
        q.w = std::cos(yaw * 0.5f);
        q.x = 0.0f;
        q.y = 0.0f;
        q.z = std::sin(yaw * 0.5f);
        return q;
    }
};

void TestFlight::send_once() {
    if (sent_) return;

    sent_ = true;
    timer_->cancel();

    if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available.");
        return;
    }

    auto goal = FlightPath::Goal();
    goal.id = 1;
    goal.pos_tol = 0.05f;
    goal.yaw_tol = 0.001f;
    goal.v_max = 1.0f;

    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id_;

    auto make_pose = [&](float x, float y, float z, float yaw) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = frame_id_;
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.position.z = z;
        ps.pose.orientation = yaw2quat(yaw);
        return ps;
    };

    path.poses.push_back(make_pose(0, 0, 5, 0));
    path.poses.push_back(make_pose(0, 1, 5, 0));
    path.poses.push_back(make_pose(0, 2, 5, 0));
    path.poses.push_back(make_pose(0, 3, 5, 0));
    path.poses.push_back(make_pose(0, 4, 5, 0));
    path.poses.push_back(make_pose(0, 5, 5, 0));
    path.poses.push_back(make_pose(0, 6, 5, 0));
    path.poses.push_back(make_pose(0, 7, 5, 0));
    path.poses.push_back(make_pose(0, 8, 5, 0));
    path.poses.push_back(make_pose(0, 9, 5, 0));
    path.poses.push_back(make_pose(0, 10, 5, 0));
    
    path.poses.push_back(make_pose(0, 11, 5, M_PI_4));
    path.poses.push_back(make_pose(3, 11, 5, M_PI_2));
    path.poses.push_back(make_pose(6, 11, 5, M_PI_2 + M_PI_4));
    
    path.poses.push_back(make_pose(6, 11, 5, M_PI));
    path.poses.push_back(make_pose(6, 10, 5, M_PI));
    path.poses.push_back(make_pose(6, 9, 5, M_PI));
    path.poses.push_back(make_pose(6, 8, 5, M_PI));
    path.poses.push_back(make_pose(6, 7, 5, M_PI));
    path.poses.push_back(make_pose(6, 6, 5, M_PI));
    path.poses.push_back(make_pose(6, 5, 5, M_PI));
    path.poses.push_back(make_pose(6, 4, 5, M_PI));
    path.poses.push_back(make_pose(6, 3, 5, M_PI));
    path.poses.push_back(make_pose(6, 2, 5, M_PI));
    path.poses.push_back(make_pose(6, 1, 5, M_PI));
    path.poses.push_back(make_pose(6, 0, 5, M_PI));




    goal.path = path;

    rclcpp_action::Client<FlightPath>::SendGoalOptions opts;
    opts.feedback_callback = 
        [this](auto, const std::shared_ptr<const FlightPath::Feedback> fb) {
            RCLCPP_INFO(this->get_logger(),
                "Feedback: idx=%u rem=%.2f",
                fb->active_idx,
                fb->dist_to_goal
            );
        };

    opts.result_callback = 
        [this](const auto& res) {
            auto code = res.code;
            const auto& r = res.result;
            RCLCPP_INFO(this->get_logger(),
                "Result: code=%d succes=%d msg=%s",
                (int)code,
                r->succes,
                r->message.c_str()
            );
        };

    RCLCPP_INFO(this->get_logger(), "Sending FlightPath goal plan_id=%lu (%zu poses)",
        (unsigned long)goal.id, goal.path.poses.size()
    );
    
    client_->async_send_goal(goal, opts);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestFlight>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}