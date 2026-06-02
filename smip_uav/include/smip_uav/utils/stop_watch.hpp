#ifndef STOP_WATCH_HPP_
#define STOP_WATCH_HPP_

#include <chrono>

struct StopWatch {
    using clock = std::chrono::high_resolution_clock;

    clock::time_point start_time;

    inline void tic() {
        start_time = clock::now();
    }

    inline double toc() {
        auto end_time = clock::now();
        std::chrono::duration<double, std::milli> elapsed = end_time - start_time;
        return elapsed.count();
    }
};

#endif