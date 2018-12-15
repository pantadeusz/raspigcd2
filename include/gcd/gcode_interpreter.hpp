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

#ifndef __RASPIGCD_GCD_GCODE_INTERPRETER_HPP__
#define __RASPIGCD_GCD_GCODE_INTERPRETER_HPP__

#include <configuration.hpp>
//#include <memory>
//#include <hardware/low_steppers.hpp>
//#include <hardware/motor_layout.hpp>
//#include <hardware/stepping.hpp>
//#include <gcd/factory.hpp>
//#include <movement/path_intent_t.hpp>

#include <movement/path_intent_t.hpp> // TODO: Remove this
#include <movement/physics.hpp>

#include <list>
#include <map>
#include <string>


namespace raspigcd {
namespace gcd {

using block_t = std::map<char, double>;             // represents N001G0X10Y20
using program_t = std::vector<block_t>;               // represents whole program without empty lines
using partitioned_program_t = std::list<program_t>; // represents program partitioned into different sections for optimization and interpretation


movement::path_intent_t generate_path_intent(const program_t& parsed_program_);


/**
 * @brief Generates string based on gcode grouped by fragments G1, G0 and M
 */
std::string back_to_gcode(partitioned_program_t &btg);


/**
 * @brief Gropus gcode commands so the other parts can focus on the simple interpretation of parts
 * 
 * It groups into G0, G1 and M codes
 */
partitioned_program_t group_gcode_commands(const program_t& program_states, const block_t & initial_state = {{'F',1}} );

/**
 * @brief Interprets gcode program to list of gcode state updates
 * 
 * for example, for
 *   G0X10
 *   G1Y2
 * will give
 *   {
 *     {{'G',0},{'X',10}},
 *     {{'G',1},{'Y',2}}
 *   }
 */
program_t gcode_to_maps_of_arguments(const std::string& program_);

block_t command_to_map_of_arguments(const std::string& command_);

} // namespace gcd
} // namespace raspigcd

#endif
