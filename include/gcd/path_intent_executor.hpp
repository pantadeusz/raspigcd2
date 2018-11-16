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

#ifndef __RASPIGCD_GCD_PATH_INTENT_EXECUTION_HPP__
#define __RASPIGCD_GCD_PATH_INTENT_EXECUTION_HPP__

#include <configuration.hpp>
#include <gcd/gcode_interpreter_objects_t.hpp>
#include <hardware/motor_layout.hpp>
#include <hardware/low_steppers.hpp>
#include <hardware/stepping.hpp>
#include <movement/path_intent_t.hpp>
#include <movement/steps_analyzer.hpp>
#include <movement/steps_generator.hpp>
#include <movement/variable_speed.hpp>

#include <chrono> // std::chrono::seconds
#include <future>
#include <map>
#include <memory>
#include <thread> // std::this_thread::sleep_for
#include <mutex>

namespace raspigcd {
namespace gcd {


class path_intent_executor
{
    std::mutex _execute_mutex;    
    gcode_interpreter_objects_t _gcdobjs;
public:

    /**
     * @brief execute path intent
     * 
     * path intent is a list of node points and transitions that make the whole execution plan.
     * 
     * The method executes the path intent. This method will run exclusively, so no other execute can be executed at the same time.
     * 
     */
    void execute(const movement::path_intent_t& path_intent);

    void set_gcode_interpreter_objects(const gcode_interpreter_objects_t &gcdobjs_) {
        _gcdobjs = gcdobjs_;
    }
    gcode_interpreter_objects_t get_gcode_interpreter_objects() const {
        return _gcdobjs;
    }
}; // namespace gcd


} // namespace gcd
} // namespace raspigcd

#endif
