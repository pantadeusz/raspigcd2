#include <hardware/driver/low_timers_fake.hpp>
#include <chrono>
#include <thread>

namespace raspigcd {
namespace hardware {
namespace driver {
void low_timers_fake::wait_s(const double )
{
    // auto prevTime = std::chrono::steady_clock::now();
    // prevTime = prevTime + std::chrono::microseconds((int)(t * 1000000.0));
    // std::this_thread::sleep_until(prevTime);
}
} // namespace driver
} // namespace hardware
} // namespace raspigcd
