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
using partitioned_program_t = std::vector<program_t>; // represents program partitioned into different sections for optimization and interpretation

/**
 * convert block to position
 * */
distance_t block_to_distance_t(const block_t& block);
/**
 * calculates movement vector from block a to block b
 * */
distance_t blocks_to_vector_move(const block_t& block_a, const block_t& block_b);


//movement::path_intent_t generate_path_intent(const program_t& parsed_program_);

/**
 * @brief  generates coordinates from block
 * 
 */
distance_t block_to_distance_t(const block_t & block);

/**
 * @brief returns the movement vector out of the two consecutive blocks
 */
distance_t blocks_to_vector_move(const block_t & block_a, const block_t & block_b);


/**
 * @brief Generates string based on gcode grouped by fragments G1, G0 and M
 */
std::string back_to_gcode(partitioned_program_t &btg);


//partitioned_program_t apply_machine_limits(
//    const partitioned_program_t& program_states,
//    // limits: acceleration, max_velocity, min_velocity_no_accel
//    const block_t & initial_state = {{'F',1}} );

// program_t apply_machine_limits (const program_t& program_states,
//                const configuration::limits &machine_limits);


/**
 * @brief Adds limits to the machine turns based on the maximal speeds and angles
 * the states receives the minimum of feedrate based on intended feedrate and the
 * maximal feedrate based on turn angle and limits
 */
program_t apply_limits_for_turns (const program_t& program_states,
                const configuration::limits &machine_limits);

program_t g0_move_to_g1_sequence (const program_t& program_states,
                const configuration::limits &machine_limits,
                block_t current_state = {{'X',0},{'Y',0},{'Z',0},{'A',0}});

/**
 * @brief enrich gcode that the moves can accelerate and break based on the limits
 * 
 * adds blocks that allow to have A->accelerate->A'->const_speed->B'->break->B
 * 
 * this itself allow for the more precise movements
 */
//program_t enrich_program_with_limits (const program_t& program_states,
//                const configuration::limits &machine_limits);


/**
 * @brief Gropus gcode commands so the other parts can focus on the simple interpretation of parts
 * 
 * It groups into G0, G1 and M codes
 */
partitioned_program_t group_gcode_commands(const program_t& program_states, const block_t & initial_state = {{'F',1}} );

/**
 * @brief updates values in destination block from source block.
 * 
 * For example, in destination there is X10 Y20, and in source is Y1 Z0, then
 * the result will be X10 Y1 Z0
 */
block_t merge_blocks(const block_t &destination, const block_t &source);

/**
 * @brief reduces gcode that only difference is left
 * 
 * @untested
 */
block_t diff_blocks(const block_t& destination, const block_t& source);

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
