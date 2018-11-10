#include <hardware/stepping.hpp>
#include <hardware/stepping_commands.hpp>
#include <hardware/thread_helper.hpp>

#include <chrono>
#include <configuration.hpp>
#include <distance_t.hpp>
#include <functional>
//#include <memory>
#include <steps_t.hpp>


#include <cstring>
#include <stdexcept>

namespace raspigcd {
namespace hardware {

std::list<steps_t> hardware_commands_to_steps(const std::vector<multistep_command>& commands_to_do)
{
    std::list<steps_t> ret;
    steps_t _steps = {0,0,0,0};
    for (const auto& s : commands_to_do) {
        for (int i = 0; i < s.cmnd.count; i++) {
            for (int j = 0; j < 4; j++)
                _steps[j] = _steps[j] + (int)((signed char)s.cmnd.b[j].step * ((signed char)s.cmnd.b[j].dir * 2 - 1));
            ret.push_back(_steps);
        }
    }
    return ret;
}


void stepping_sim::exec(const std::vector<multistep_command>& commands_to_do)
{
    _tick_index = 0;
    auto start_steps = current_steps;
    for (auto& steps : hardware_commands_to_steps(commands_to_do)) {
        current_steps = steps + start_steps;
        _on_step(steps); // callback virtually set
        _tick_index++;
    }
}

void stepping_simple_timer::set_delay_microseconds(int delay_ms)
{
    _delay_microseconds = delay_ms;
}

void stepping_simple_timer::set_low_level_steppers_driver(std::shared_ptr<low_steppers> steppers_driver)
{
    _steppers_driver_shr = steppers_driver;
    _steppers_driver = _steppers_driver_shr.get();
}

void stepping_simple_timer::exec(const std::vector<multistep_command>& commands_to_do)
{
    set_thread_realtime();
    _tick_index = 0;
    //steps_t _steps = start_steps;
    std::chrono::microseconds ttime = std::chrono::microseconds((unsigned long)(_delay_microseconds));
    auto t = std::chrono::system_clock::now();
    auto nextT = ttime + t;
    long int step_number = 0;
    nextT = t;
    for (const auto& s : commands_to_do) {
        for (int i = 0; i < s.cmnd.count; i++) {
//             _steps[0] = _steps[0] + (int)((signed char)s.cmnd.b[0].step * ((signed char)s.cmnd.b[0].dir * 2 - 1));
//             _steps[1] = _steps[1] + (int)((signed char)s.cmnd.b[1].step * ((signed char)s.cmnd.b[1].dir * 2 - 1));
//             _steps[2] = _steps[2] + (int)((signed char)s.cmnd.b[2].step * ((signed char)s.cmnd.b[2].dir * 2 - 1));
//             _steps[3] = _steps[3] + (int)((signed char)s.cmnd.b[3].step * ((signed char)s.cmnd.b[3].dir * 2 - 1));
            ttime = std::chrono::microseconds((unsigned long)(_delay_microseconds));
            _steppers_driver->do_step(s.cmnd.b);
            _tick_index++;
//             on_step_(_steps);
            // wait till next step
            nextT += ttime;
            step_number++;
//nextT = t + ttime*step_number;
// always busy wait - better timing, but more resource consuming
#ifndef STEPPING_DELAY_SLEEP_UNTIL
            for (; std::chrono::system_clock::now() < nextT;)
                ;
#else
            std::this_thread::sleep_until(nextT);
#endif
        }
    }
    //return _steps;
}
/*

{
    configuration_t& conf = configuration;
    auto steppers = conf.hardware.steppers;
    // commands are in fomat step, dir
    unsigned int step_clear = (1 << steppers[0].step) | (1 << steppers[1].step) |
                              (1 << steppers[2].step) | (1 << steppers[3].step);

    std::chrono::microseconds ttime =
        std::chrono::microseconds((unsigned long)(conf.tick_duration / 0.000001));
    auto t = std::chrono::system_clock::now();
    auto nextT = ttime + t;
    int current_tick_n = 0;

    // this part is critical - I unwinded loops in order to reduce latencies
    for (auto c : commands) {
        int rpt = c.cmnd.count; // 0 means that we execute it once
        do {
            execute_single_tick(c, steppers, _position, step_clear);
            //nextT = t + ttime * current_tick_n;
            nextT += ttime;
            current_tick_n++;
            // std::this_thread::sleep_until(nextT);
            // always busy wait - better timing, but more resource consuming
            for (; std::chrono::system_clock::now() < nextT;) {
            }
        } while ((rpt--) > 0);
        if (_terminate) {
            _terminate = false;
            return 1;
        }
    }

}
 */
} // namespace hardware
} // namespace raspigcd
