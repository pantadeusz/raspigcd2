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

#ifndef __RASPIGCD_MOVEMENT_MOVEMENT_PLAN_T_T_HPP__
#define __RASPIGCD_MOVEMENT_MOVEMENT_PLAN_T_T_HPP__

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

struct transition_t {
    double v0;    // initial velocity
    double accel; // acceleration to the next node
    double max_v; // maximal intended velocity that can be performed on this fragment. The most desired speed. The velocity cannot exceed max_v no matter what.
};

using delay_t = double; // delay in seconds
using movement_plan_element_t = std::variant<distance_t, transition_t, delay_t>;
using movement_plan_t = std::list<movement_plan_element_t>;

} // namespace movement
} // namespace raspigcd

#endif
