#include "ros_nodes/frame_node.hpp"

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

    auto frame_node = std::make_shared<FrameNode>();

    rclcpp::spin(frame_node);
    rclcpp::shutdown();

    return 0;
}

