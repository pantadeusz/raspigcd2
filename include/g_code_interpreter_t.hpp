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


#include <chrono>
#include <configuration_t_json.hpp>
#include <distance_t.hpp>
#include <step_sequence_executor_t.hpp>
#include <buttons_t.hpp>
#include <iostream>
#include <list>
#include <memory>
#include <motion_plan_t.hpp>
#include <motor_layout_t.hpp>
#include <mutex>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>
#include <chrono>
#include <thread>

#ifndef __RASPIGCD_GCODE_INTERPRETER_HPP__
#define __RASPIGCD_GCODE_INTERPRETER_HPP__

#include <generic_spindle_t.hpp>


namespace raspigcd {
class g_code_interpreter_t
{
protected:

    std::map<int, double> _gspeed; // current speeds
    double _fr_multiplier; //  // 1 - mm/s  or  60 - mm/m

    configuration_t* _cfg; // configuraion
    step_sequence_executor_t* _executor; // the executor - the class that executes steps on motors
    std::vector < std::shared_ptr < generic_spindle_t > > _spindles;
    std::mutex _execution_of_gcode_m;
    std::map<char, std::function<std::string(const std::map<char, double>&)>> _gcode_commands; // functions handling different gcode commands 

    motion_plan_t* _motion_planner; // this handles current position

public:
    // executes gcode command
    std::string g(const std::string& gcd, std::function<std::string(const char, const std::map<char, double>&)> callback_);
    // executes gcode commands gieven as lines
    std::list<std::string> execute_gcode_lines(std::list<std::string> lines);
    // exececute whole gcode program written as string
    std::list<std::string> execute_gcode_string(const std::string gcode_text_);
    // the interpreter
    g_code_interpreter_t(configuration_t* cfg, step_sequence_executor_t* executor, std::vector < std::shared_ptr < generic_spindle_t > > spindles, motion_plan_t* motion_planner_);
};



}

#endif

