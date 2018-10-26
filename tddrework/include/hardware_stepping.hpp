/*
    Raspberry Pi G-CODE interpreter
    Copyright (C) 2018  Tadeusz Puźniakowski puzniakowski.pl
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __RASPIGCD_HARDWARE_STEPPING_T_HPP__
#define __RASPIGCD_HARDWARE_STEPPING_T_HPP__

#include <configuration.hpp>
#include <distance_t.hpp>
#include <steps_t.hpp>
//
#include <memory>
#include <functional>

namespace raspigcd {
namespace hardware {

struct single_step_command {
    unsigned char step : 1, dir : 1;
};

union multistep_command {
    struct {
        single_step_command b[RASPIGCD_HARDWARE_DOF]; // duplicate of first b
        int repeat;               // number of times to repeat the command, it means that the command will be executed (repeat+1) times.
    } cmnd;
    int64_t v;
};


class stepping
{
public:
/**
 * @brief Executes multistep commands list. On each step it calls on_step_ function with current position as an argument. The return value is the final steps value
 * 
 * @param start_steps the initial position of motors in steps
 * @param commands_to_do array of commands to execute
 * @param on_step_ function to execute on each step. This function can throw exceptions to break execution of steps
 * @return steps_t final position of the machine in steps
 */
    virtual steps_t exec(const steps_t &start_steps, const std::vector<multistep_command> &commands_to_do, std::function<void(const steps_t &)> on_step_ = [](const steps_t &){}) = 0;
};

class stepping_sim: public stepping
{
public:
    steps_t exec(const steps_t &start_steps, const std::vector<multistep_command> &commands_to_do, std::function<void(const steps_t &)> on_step_) {
        steps_t _steps = start_steps;
        for (const auto & s: commands_to_do) {
            for (int i = 0; i < s.cmnd.repeat; i++) {
                _steps[0] = _steps[0] + (int)((signed char)s.cmnd.b[0].step * ((signed char)s.cmnd.b[0].dir * 2 - 1));
                _steps[1] = _steps[1] + (int)((signed char)s.cmnd.b[1].step * ((signed char)s.cmnd.b[1].dir * 2 - 1));
                _steps[2] = _steps[2] + (int)((signed char)s.cmnd.b[2].step * ((signed char)s.cmnd.b[2].dir * 2 - 1));
                _steps[3] = _steps[3] + (int)((signed char)s.cmnd.b[3].step * ((signed char)s.cmnd.b[3].dir * 2 - 1));
                on_step_(_steps);
            }
        }
        return _steps;
    }
};


} // namespace hardware
} // namespace raspigcd

#endif
