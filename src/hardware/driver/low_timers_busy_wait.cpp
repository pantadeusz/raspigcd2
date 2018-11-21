#include <chrono>
#include <hardware/driver/low_timers_busy_wait.hpp>
#include <thread>

namespace raspigcd {
namespace hardware {
namespace driver {
void low_timers_busy_wait::wait_s(const double t)
{
    // auto prevTime = std::chrono::steady_clock::now();
    // prevTime = prevTime + std::chrono::microseconds((int)(t * 1000000.0));
    //std::this_thread::sleep_until(prevTime);
    std::this_thread::sleep_for(std::chrono::microseconds((int)(t * 1000000.0)));
}


/**
     * @brief start the timer
     * 
     */
std::chrono::high_resolution_clock::time_point low_timers_busy_wait::start_timing()
{
    return std::chrono::system_clock::now();
};

/**
     * @brief wait for the tick to end.
     * Remember to run start_timing first!
     */
std::chrono::high_resolution_clock::time_point low_timers_busy_wait::wait_for_tick_s(const std::chrono::high_resolution_clock::time_point& prev_timer, const double t)
{
    auto ttime = std::chrono::microseconds((unsigned long)(t));
    auto nextT = prev_timer + ttime;
    //nextT = t + ttime*step_number;
    // always busy wait - better timing, but more resource consuming
    for (; std::chrono::system_clock::now() < nextT;)
        ;
    //            std::this_thread::sleep_until(nextT);

    return nextT;
};

} // namespace driver
} // namespace hardware
} // namespace raspigcd
