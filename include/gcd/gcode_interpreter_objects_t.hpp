//gcode_interpreter_objects_t;


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

#ifndef __RASPIGCD_GCD_HARDWARE_GCODE_INTERPRETER_OBJECTS_T_HPP__
#define __RASPIGCD_GCD_HARDWARE_GCODE_INTERPRETER_OBJECTS_T_HPP__

#include <configuration.hpp>
#include <memory>
#include <hardware/motor_layout.hpp>
#include <hardware/low_buttons.hpp>
#include <hardware/low_steppers.hpp>
#include <hardware/low_spindles_pwm.hpp>
#include <hardware/low_timers.hpp>
#include <hardware/stepping.hpp>

namespace raspigcd {
namespace gcd {

struct gcode_interpreter_objects_t {
std::shared_ptr<hardware::low_buttons> buttons;
std::shared_ptr<hardware::low_steppers> steppers;
std::shared_ptr<hardware::low_spindles_pwm> spindles_pwm;
std::shared_ptr<hardware::low_timers> timers;
std::shared_ptr<hardware::motor_layout> motor_layout;
std::shared_ptr<hardware::stepping> stepping;
configuration::global configuration;
};

} // namespace hardware
} // namespace raspigcd

#endif
