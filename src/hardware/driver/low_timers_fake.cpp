#include <hardware/driver/low_timers_fake.hpp>
#include <chrono>
#include <thread>

namespace raspigcd {
namespace hardware {
namespace driver {
void low_timers_fake::wait_s(const double t)
{
    last_delay = t;
    on_wait_s(t);
    // auto prevTime = std::chrono::steady_clock::now();
    // prevTime = prevTime + std::chrono::microseconds((int)(t * 1000000.0));
    // std::this_thread::sleep_until(prevTime);
}

/**
     * @brief start the timer
     * 
     */
std::chrono::high_resolution_clock::time_point low_timers_fake::start_timing()
{
    return std::chrono::system_clock::now();
};

/**
     * @brief wait for the tick to end.
     * Remember to run start_timing first!
     */
std::chrono::high_resolution_clock::time_point low_timers_fake::wait_for_tick_s(const std::chrono::high_resolution_clock::time_point&, const double)
{
    return std::chrono::system_clock::now();
};
} // namespace driver
} // namespace hardware
} // namespace raspigcd
