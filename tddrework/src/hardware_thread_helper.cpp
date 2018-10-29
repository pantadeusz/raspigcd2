#include <hardware_thread_helper.hpp>

#include <thread>
#include <iostream>
#include <cerrno>
#include <cstring>
#include <stdexcept>


namespace raspigcd {
namespace hardware {

void set_thread_realtime()
{
    sched_param sch_params;
    sch_params.sched_priority = sched_get_priority_max(SCHED_RR);
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &sch_params)) {
        static int already_warned = 0;
        if (already_warned == 0) {
            std::cerr << "Warning: Failed to set Thread scheduling : "
                  << std::strerror(errno) << std::endl;
            already_warned ++;
        }
    }
}

} // namespace hardware
} // namespace raspigcd
