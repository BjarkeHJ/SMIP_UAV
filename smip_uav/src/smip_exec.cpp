#include "ros_nodes/map_node.hpp"
#include "ros_nodes/planner_node.hpp"
#include "ros_nodes/voxl_tf_node.hpp"

#include <pthread.h>
#include <sched.h>
#include <cerrno>
#include <cstring>
#include <cstdio>

using namespace smip_uav;

int main(int argc, char** argv) {
    struct sched_param sp{};
    sp.sched_priority = 10;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
        fprintf(stderr, "[smip_exec] Failed to set RT priority: %s\n", strerror(errno));
    }

    rclcpp::init(argc, argv);

    auto map_node    = std::make_shared<SurfelMapNode>();
    auto ov_tf_node  = std::make_shared<VoxlTfNode>();

    // auto planner_node = std::make_shared<>();
    // planner_node->set_map(map_node->get_map());

    rclcpp::executors::MultiThreadedExecutor smip_exec;
    smip_exec.add_node(map_node);
    smip_exec.add_node(ov_tf_node);
    // smip_exec.add_node(planner_node);
    
    smip_exec.spin();

    rclcpp::shutdown();
    return 0;
}
