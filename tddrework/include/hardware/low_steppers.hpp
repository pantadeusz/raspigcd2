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

#ifndef __RASPIGCD_HARDWARE_LOW_LEVEL_STEPPERS_T_HPP__
#define __RASPIGCD_HARDWARE_LOW_LEVEL_STEPPERS_T_HPP__

#include <configuration.hpp>
#include <hardware/stepping_commands.hpp>

#include <vector>

namespace raspigcd {
namespace hardware {


class low_steppers {
private:
public:
    /**
     * @brief execute single step command. That is the single most basic "step-dir" action
     * 
     * @param b step-dir for every stepper motor (depends on the )
     */
    virtual void do_step(const single_step_command *b) = 0;

    /**
     * @brief turn on or off the stepper motors. If the hardware supports it, then
     *        each motor can be enabled independently
     * 
     * @param en enable status for each motor
     * @return raspberry_pi_3&  returns this object
     */
    virtual void enable_steppers(const std::vector<bool> en) = 0;
};

} // namespace hardware
} // namespace raspigcd

#endif
