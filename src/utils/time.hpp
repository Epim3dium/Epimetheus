#ifndef EPI_TIME_HPP
#define EPI_TIME_HPP
#include <chrono>
namespace epi {

class Time {
    double delta_time;
    typedef  std::chrono::duration<double> duration_t;
    typedef std::chrono::time_point<std::chrono::steady_clock> time_point_t;

    time_point_t start;
    time_point_t stop;
    static Time& getInstance() {
        static Time s_Instance;
        return s_Instance;
    }
public:
    static void update() {
        getInstance().stop = std::chrono::steady_clock::now();

        auto dur = getInstance().stop - getInstance().start;
        getInstance().start = std::chrono::steady_clock::now();
        getInstance().delta_time = 
            static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(dur).count()) / 1e9;
    }
    static double deltaTime() {
        return getInstance().delta_time;
    }
};

}
#endif// EPI_TIME_HPP
