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

#ifndef __RASPIGCD_MOVEMENT_SIMPLE_STEPS_T_HPP__
#define __RASPIGCD_MOVEMENT_SIMPLE_STEPS_T_HPP__

#include <configuration.hpp>
#include <distance_t.hpp>
#include <hardware/stepping_commands.hpp>
#include <list>
#include <steps_t.hpp>

namespace raspigcd {
namespace movement {

namespace simple_steps {

int steps_remaining(const steps_t& steps_, const steps_t& destination_steps_);

/**
 * @brief generates steps to reach given destination steps
 * @arg steps_ current steps count
 * @arg destination_steps_ desired steps count
 */
std::vector<hardware::multistep_command> chase_steps(const steps_t& steps_, steps_t destination_steps_);


} // namespace simple_steps

} // namespace movement
} // namespace raspigcd

#endif
