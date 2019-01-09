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

#ifndef __CONVERTERS_GCD_PROGRAM_TO_STEPS_HPP___
#define __CONVERTERS_GCD_PROGRAM_TO_STEPS_HPP___


#include <gcd/gcode_interpreter.hpp>
#include <hardware/stepping_commands.hpp>
#include <hardware_dof_conf.hpp>
#include <functional>

namespace raspigcd {
namespace converters {

hardware::multistep_commands_t program_to_steps(
    const gcd::program_t& prog_,
    const configuration::actuators_organization& conf_,
    hardware::motor_layout &ml_,
    const gcd::block_t& initial_state_ = {},
    std::function<void(const gcd::block_t &)> finish_callback_f_ = [](const gcd::block_t &){});

} // namespace converters
} // namespace raspigcd


#endif
