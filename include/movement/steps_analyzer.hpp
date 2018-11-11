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

#ifndef __RASPIGCD_MOVEMENT_STEPS_ANALYZER_T_HPP__
#define __RASPIGCD_MOVEMENT_STEPS_ANALYZER_T_HPP__

#include <cmath>
#include <configuration.hpp>
#include <distance_t.hpp>
#include <hardware/motor_layout.hpp>
#include <hardware/stepping_commands.hpp>
#include <memory>
#include <movement/simple_steps.hpp>
#include <steps_t.hpp>
#include <variant>
#include <functional>


namespace raspigcd {
namespace movement {

//struct transition_t {
//    double v0;    // initial velocity
//    double accel; // acceleration to the next node
//    double max_v; // maximal intended velocity that can be performed on this fragment. The most desired speed. The velocity cannot exceed max_v no matter what.
//};

//using movement_plan_t = std::list<std::variant<distance_t, transition_t>>;

class steps_analyzer
{
protected:
    hardware::motor_layout* _motor_layout;
    std::shared_ptr<hardware::motor_layout> _motor_layout_ptr;

public:
    void set_motor_layout(std::shared_ptr<hardware::motor_layout> ml) {
        _motor_layout = ml.get();
        _motor_layout_ptr = ml;
    };

    steps_analyzer(std::shared_ptr<hardware::motor_layout> ml) {
        set_motor_layout(ml);
    };

    /**
     * @brief Returns the position in steps _after_ the execution of given numbers of tick.
     * 
     * Note that if the tick number is 0  then no ticks are executed.
     * 
     */
    steps_t steps_from_tick(const std::vector<hardware::multistep_command> &commands_to_do,const int tick_number) const {
        auto tick_number_ = tick_number;
        steps_t _steps = {0,0,0,0};
        int cmnd_i = 0;
        int i = 0;
        for (const auto& s : commands_to_do) {
            //std::cout << " i " << i << " cmnd_i " << cmnd_i << std::endl;
            if ((tick_number_ >= i) && (tick_number_ < (i + s.cmnd.count))) {
                //std::cout << " i " << i << " cmnd_i " << cmnd_i << "  >> " << (tick_number_ - i) << std::endl;
                for (int j = 0; j < 4; j++)
                    _steps[j] = _steps[j] + (tick_number_ - i)*((int)((signed char)s.cmnd.b[j].step * ((signed char)s.cmnd.b[j].dir * 2 - 1)));
                return _steps;
            } else {
                for (int j = 0; j < 4; j++)
                    _steps[j] = _steps[j] + s.cmnd.count*((int)((signed char)s.cmnd.b[j].step * ((signed char)s.cmnd.b[j].dir * 2 - 1)));
            }
            cmnd_i++;
            i += s.cmnd.count;
        }
        if (i == tick_number_) 
            return _steps;
        throw std::out_of_range("the index is after the last step");
    };
    int get_last_tick_index(const std::vector<hardware::multistep_command> &commands_to_do) const {
        int cmnd_i = 0;
        int i = 0;
        for (const auto& s : commands_to_do) {
            cmnd_i++;
            i += s.cmnd.count;
        }
        return i;
    };


};

} // namespace movement
} // namespace raspigcd

#endif
