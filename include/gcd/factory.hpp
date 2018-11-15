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

#ifndef __RASPIGCD_GCD_FACTORY_T_HPP__
#define __RASPIGCD_GCD_FACTORY_T_HPP__

#include <configuration.hpp>
#include <memory>
#include <hardware/low_steppers.hpp>
#include <hardware/motor_layout.hpp>
#include <hardware/stepping.hpp>

namespace raspigcd {
namespace gcd {

class gcode_interpreter_objects {
public:
std::shared_ptr<hardware::low_steppers> hardware_driver;
std::shared_ptr<hardware::motor_layout> motor_layout;
std::shared_ptr<hardware::stepping> stepping;
configuration::global configuration;
};

enum machine_driver_selection {
    RASPBERRY_PI,
    IN_MEMORY
};

gcode_interpreter_objects gcode_interpreter_objects_factory(const configuration::global &configuration_, const machine_driver_selection driver_select_ = RASPBERRY_PI);

} // namespace hardware
} // namespace raspigcd

#endif
