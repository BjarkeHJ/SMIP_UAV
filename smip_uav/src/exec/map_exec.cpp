#include "ros_nodes/active_map_node.hpp"

#include <memory>
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

    auto container = std::make_shared<MapStateContainer>();
    auto active_map_node = std::make_shared<ActiveMapNode>(container);

    rclcpp::executors::MultiThreadedExecutor map_exec;
    map_exec.add_node(active_map_node);
    map_exec.spin();

    rclcpp::shutdown();
    return 0;
}