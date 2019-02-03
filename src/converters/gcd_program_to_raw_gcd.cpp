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


#include <converters/gcd_program_to_raw_gcd.hpp>
#include <movement/physics.hpp>
#include <movement/simple_steps.hpp>

#include <functional>

namespace raspigcd {
namespace converters {

gcd::program_t program_to_raw_program (
    const gcd::program_t& prog_,
    const configuration::actuators_organization& conf_,
    hardware::motor_layout& ml_,
    const gcd::block_t& initial_state_)
{
    return {};
}


} // namespace converters
} // namespace raspigcd
