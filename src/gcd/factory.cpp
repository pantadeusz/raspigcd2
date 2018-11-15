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

#include <hardware/motor_layout.hpp>
#include <hardware/stepping_commands.hpp>
#include <movement/simple_steps.hpp>
#include <movement/variable_speed.hpp>
#include <memory>

namespace raspigcd {
namespace gcd {

class gcode_engine_objects{
    public:
    
};

class gcode_objects_factory {
public:

};


} // namespace gcd
} // namespace raspigcd
