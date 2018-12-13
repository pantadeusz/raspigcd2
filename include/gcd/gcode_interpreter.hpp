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

#include <movement/path_intent_t.hpp>

#include <string>
#include <map>
#include <list>


namespace raspigcd {
namespace gcd {

movement::path_intent_t generate_path_intent(const std::list< std::map<char,double> > &parsed_program_);

std::list< std::map<char,double> > gcode_to_maps_of_arguments(const std::string &program_);
std::map<char,double> command_to_map_of_arguments(const std::string &command_);

} // namespace hardware
} // namespace raspigcd

#endif
