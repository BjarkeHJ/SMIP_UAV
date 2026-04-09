#include "ros_nodes/trajectory_node.hpp"
#include <cmath>

TrajectoryNode::TrajectoryNode() : Node("trajectory_node") {
    target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/gz_px4_sim/target_pose", 5);

    server_ = rclcpp_action::create_server<FlightPath>(
        this, "flight_path",
        std::bind(&TrajectoryNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TrajectoryNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&TrajectoryNode::handle_accepted, this, std::placeholders::_1));
    
    tick_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(tick_ms_),
        std::bind(&TrajectoryNode::execute_tick, this));
    
        RCLCPP_INFO(this->get_logger(), "TrajectoryNode ActionServer ready.");
}

rclcpp_action::GoalResponse TrajectoryNode::handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const FlightPath::Goal> goal) {
    if (goal->path.poses.size() < 1) {
        RCLCPP_WARN(this->get_logger(), "Rejected goal: empty path.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (active_gh_) {
        RCLCPP_WARN(this->get_logger(), "Rejected goal: another goal is active.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "Accepted goal id=%lu  %zu waypoints  v_max=%.2f",
        (unsigned long)goal->id, goal->path.poses.size(), goal->v_max
    );

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrajectoryNode::handle_cancel(std::shared_ptr<GoalHandle>) {
    RCLCPP_INFO(this->get_logger(), "Cancel requested.");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TrajectoryNode::handle_accepted(std::shared_ptr<GoalHandle> gh) {
    active_gh_ = gh;
    const auto& goal = *gh->get_goal();

    waypoints_.clear();
    for (const auto& ps : goal.path.poses) {
        Waypoint wp;
        wp.pos = Eigen::Vector3f(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
        wp.yaw = quat2yaw(ps.pose.orientation);
        waypoints_.push_back(wp);
    }

    seg_idx_ = 0;
    pos_tol_ = goal.pos_tol;
    yaw_tol_ = goal.yaw_tol;
    v_max_ = goal.v_max;

    interp_pos_ = waypoints_[0].pos;
    interp_yaw_ = waypoints_[0].yaw;

    if (!goal.path.header.frame_id.empty()) {
        frame_id_ = goal.path.header.frame_id;
    }
}

void TrajectoryNode::execute_tick() {
    if (!active_gh_) return;

    if (active_gh_->is_canceling()) {
        auto result = std::make_shared<FlightPath::Result>();
        result->succes = false;
        result->message = "Cancelled";
        result->final_idx = seg_idx_;
        active_gh_->canceled(result);
        active_gh_ = nullptr;
        RCLCPP_INFO(this->get_logger(), "Goal cancelled.");
        return;
    }

    const uint32_t last_idx = waypoints_.size() - 1;

    if (seg_idx_ <= last_idx) {
        const Waypoint& tgt = waypoints_[seg_idx_];

        // Interpolate position
        Eigen::Vector3f dp = tgt.pos - interp_pos_;
        float dist = dp.norm();

        float step = static_cast<float>(v_max_ * k_dt_);
        if (dist > step) {
            interp_pos_ += dp.normalized() * step;
        }
        else {
            interp_pos_ = tgt.pos;
        }

        // Interpolate yaw
        float yaw_err = shortest_yaw_diff(interp_yaw_, tgt.yaw);
        if (dist > step) {
            float t_remain = dist / v_max_;
            float yaw_rate = (t_remain > 1e-6f) ? yaw_err / t_remain : yaw_err;
            float yaw_step = yaw_rate * static_cast<float>(k_dt_);
            if (std::fabs(yaw_step) > std::fabs(yaw_err)) {
                yaw_step = yaw_err;
            }
            interp_yaw_ += yaw_step;
        }
        else {
            interp_yaw_ = tgt.yaw;
        }

        interp_yaw_ = wrap_pi(interp_yaw_);

        // Check waypoint reached
        float pos_err = (tgt.pos - interp_pos_).norm();
        float yaw_err_abs = std::fabs(shortest_yaw_diff(interp_yaw_, tgt.yaw));

        if (pos_err <= pos_tol_ && yaw_err_abs <= yaw_tol_) {
            RCLCPP_INFO(this->get_logger(), "Waypoint %u reached.", seg_idx_);
            if (seg_idx_ < last_idx) {
                seg_idx_++;
            }
            else {
                auto result = std::make_shared<FlightPath::Result>();
                result->succes = true;
                result->message = "Completed";
                result->final_idx = seg_idx_;
                active_gh_->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal completed.");
                active_gh_ = nullptr;
                return;
            }
        }
    }

    target_pub_->publish(make_target_msg());

    auto fb = std::make_shared<FlightPath::Feedback>();
    fb->active_idx = seg_idx_;
    fb->current_pose = make_target_msg();
    fb->dist_to_goal = (waypoints_.back().pos - interp_pos_).norm();
    active_gh_->publish_feedback(fb);
}

float TrajectoryNode::wrap_pi(float a) {
    while (a > M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

float TrajectoryNode::shortest_yaw_diff(float from, float to) {
    return wrap_pi(to - from);
}

geometry_msgs::msg::Quaternion TrajectoryNode::yaw2quat(float yaw) {
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(yaw * 0.5f);
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = std::sin(yaw * 0.5f);
    return q;
}

float TrajectoryNode::quat2yaw(const geometry_msgs::msg::Quaternion& q) {
    // standard ZYX YAW: atan2(2*(wz+xy), 1-2(yy+zz))
    float siny = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    return std::atan2(siny, cosy);
}

geometry_msgs::msg::PoseStamped TrajectoryNode::make_target_msg() {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = this->get_clock()->now();
    ps.header.frame_id = frame_id_;
    ps.pose.position.x = interp_pos_.x();
    ps.pose.position.y = interp_pos_.y();
    ps.pose.position.z = interp_pos_.z();
    ps.pose.orientation = yaw2quat(interp_yaw_);
    return ps;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}