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
        for (int i = 0; i < s.count; i++) {
            for (int j = 0; j < 4; j++)
                _steps[j] = _steps[j] + (int)((signed char)s.b[j].step * ((signed char)s.b[j].dir * 2 - 1));
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

void stepping_simple_timer::set_low_level_timers(std::shared_ptr<low_timers> timer_drv_) {
    _low_timer_shr = timer_drv_;
    _low_timer = timer_drv_.get();
}


void stepping_simple_timer::exec(const std::vector<multistep_command>& commands_to_do)
{
    set_thread_realtime();
    _tick_index = 0;
    //steps_t _steps = start_steps;
//    std::chrono::microseconds ttime = std::chrono::microseconds((unsigned long)(_delay_microseconds));
//    auto t = std::chrono::system_clock::now();
//    auto nextT = ttime + t;
    //long int step_number = 0;
//    nextT = t;
    std::chrono::high_resolution_clock::time_point prev_timer = _low_timer->start_timing();
    for (const auto& s : commands_to_do) {
        for (int i = 0; i < s.count; i++) {
//             _steps[0] = _steps[0] + (int)((signed char)s.b[0].step * ((signed char)s.b[0].dir * 2 - 1));
//             _steps[1] = _steps[1] + (int)((signed char)s.b[1].step * ((signed char)s.b[1].dir * 2 - 1));
//             _steps[2] = _steps[2] + (int)((signed char)s.b[2].step * ((signed char)s.b[2].dir * 2 - 1));
//             _steps[3] = _steps[3] + (int)((signed char)s.b[3].step * ((signed char)s.b[3].dir * 2 - 1));
//            ttime = std::chrono::microseconds((unsigned long)(_delay_microseconds));
            _steppers_driver->do_step(s.b);
            _tick_index++;
            

        prev_timer = _low_timer->wait_for_tick_s(prev_timer, _delay_microseconds);

//             on_step_(_steps);
            // wait till next step
//            nextT += ttime;
            //step_number++;
//nextT = t + ttime*step_number;
// always busy wait - better timing, but more resource consuming
//#ifndef STEPPING_DELAY_SLEEP_UNTIL
//            for (; std::chrono::system_clock::now() < nextT;)
//                ;
//#else
//            std::this_thread::sleep_until(nextT);
//#endif
        }
    }
    //return _steps;
}

} // namespace hardware
} // namespace raspigcd
