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

#ifndef __RASPIGCD_GCD_GCODE_MOVES_TO_INTENTIONS_HPP__
#define __RASPIGCD_GCD_GCODE_MOVES_TO_INTENTIONS_HPP__

#include <configuration.hpp>
#include <memory>
#include <hardware/low_steppers.hpp>
#include <hardware/motor_layout.hpp>
#include <hardware/stepping.hpp>
#include <gcd/factory.hpp>
#include <movement/path_intent_t.hpp>

#include <map>

namespace raspigcd {
namespace gcd {


class gcode_interpreter_state {
public:
    // the head position that is visible for gcode interpreter
    distance_t current_position;
    std::vector<std::string> gcode; // current executing gcode
};

class interpreter {
protected:

    gcode_interpreter_objects _gcdobjs;
    gcode_interpreter_state _state;

public:


    /**
     * @brief interprets the single G command.
     * 
     * @param m is a map that contains symbol and value.
     * 
     * For example G0X10 will give the following map: {{'G',0},{'X',10}}
     */

    std::string codename_g(const std::map<char, double>& m){
        throw "TODO";
    };

    std::list < std::string > interpret_gcode(const std::vector<std::string> &gcode_) {
        throw "TODO";
    }

    /**
     * @brief the setup in necessary for this class to work
     * 
     * @param gcdobjs_ the setup of the machine that is ready to run
     */
    
    interpreter(gcode_interpreter_objects &gcdobjs_) {
        _gcdobjs = gcdobjs_;
    }

};

/**
 * @brief this function interprets commands like G0, G1, ... and returns path_intentions
 * 
 */
movement::path_intent_t movement_commands_to_paths(const std::vector < std::string > gcode_lines, gcode_interpreter_objects &gcdobjs) {}

} // namespace hardware
} // namespace raspigcd

#endif
