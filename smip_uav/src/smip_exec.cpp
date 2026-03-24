#include "ros_nodes/map_node.hpp"
#include "ros_nodes/planner_node.hpp"

using namespace smip_uav;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // TODO
    auto map_node = std::make_shared<SurfelMapNode>();
    // auto planner_node = std::make_shared<>();

    // planner_node->set_map(map_node->get_map());

    rclcpp::executors::MultiThreadedExecutor smip_exec;
    smip_exec.add_node(map_node);
    // smip_exec.add_node(planner_node);
    smip_exec.spin();

    rclcpp::shutdown();
    return 0;
}
