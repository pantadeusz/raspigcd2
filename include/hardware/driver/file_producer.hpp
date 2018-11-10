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

#ifndef __RASPIGCD_HARDWARE_DRIVER_FILE_PRODUCER_T_HPP__
#define __RASPIGCD_HARDWARE_DRIVER_FILE_PRODUCER_T_HPP__


#include <configuration.hpp>
#include <distance_t.hpp>
#include <hardware/low_buttons.hpp>
#include <hardware/low_spindles_pwm.hpp>
#include <hardware/low_steppers.hpp>
#include <hardware/stepping_commands.hpp>
#include <steps_t.hpp>

#include <functional>
#include <map>
#include <memory>
#include <thread>

namespace raspigcd {
namespace hardware {
namespace driver {


/**
 * (WORK IN PROGRESS)
 *
 * THIS CLASS IS NOT READY YET. It will not be tested in the forseenable future.
 * 
 * This is for collecting statistical data about execution of program. Can record times and coordinates. 
 * */

class file_producer :  public low_steppers//, public low_buttons, public low_spindles_pwm
{
public:
    int counters[RASPIGCD_HARDWARE_DOF];
    std::vector<bool> enabled;

    void on_step(){};


////////// low_steppers //////////

    void do_step(const single_step_command* b)
    {
        for (int i = 0; i < RASPIGCD_HARDWARE_DOF; i++) {
            if (enabled[i]) if (b[i].step == 1) {
                counters[i] += b[i].dir * 2 - 1;
            }
        }
    };
    
    void enable_steppers(const std::vector<bool> en)
    {
        enabled = en;
    };


    file_producer(const std::string &coordinates_record_file_name_)
    {
        for (int i = 0; i < RASPIGCD_HARDWARE_DOF; i++) {
            counters[i] = 0;
        }
        enabled = std::vector<bool>(false, RASPIGCD_HARDWARE_DOF);
    }
};

} // namespace visualization
} // namespace hardware
} // namespace raspigcd

#endif
