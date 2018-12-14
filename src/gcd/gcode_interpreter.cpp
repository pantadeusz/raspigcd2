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

#include <gcd/gcode_interpreter.hpp>

//#include <memory>
//#include <hardware/low_steppers.hpp>
//#include <hardware/motor_layout.hpp>
//#include <hardware/stepping.hpp>
//#include <gcd/factory.hpp>
//#include <movement/path_intent_t.hpp>
#include <iostream>
#include <iterator>
#include <map>
#include <regex>
#include <stdexcept>
#include <string>

namespace raspigcd {
namespace gcd {

movement::path_intent_t generate_path_intent(const std::list< std::map<char,double> > &parsed_program_) {
    movement::path_intent_t ret;
    std::map<char,double> g_state = {
        {'X',0.0},{'Y',0.0},{'Z',0.0},{'A',0.0}
    };
    auto get_pos = [&](){return distance_t{
        g_state['X'], g_state['Y'], g_state['Z'], g_state['A']
        }; };
    for (const auto &elem : parsed_program_) {

    }
    return {};
}


std::list< std::map<char,double> > gcode_to_maps_of_arguments(const std::string &program_) {
    // command_to_map_of_arguments
    std::list< std::map<char,double> > ret;
    std::regex re("[\r\n]");
    std::sregex_token_iterator
        first{program_.begin(), program_.end(), re, -1},
        last;
    for (auto line : std::vector<std::string>(first, last)) {
        auto cm = command_to_map_of_arguments(line);
        if (cm.size()) ret.push_back(cm);
    }
    return ret;
}


std::map<char, double> command_to_map_of_arguments(const std::string& command__)
{
    //static std::regex command_rex("[^a-zA-Z0-9.\\-;]");
    static std::regex command_rex("[ \r\n\t]");
    for (auto c : command__)
        if (c == '\n') throw std::invalid_argument("new line is not allowed");
    std::map<char, std::string> ret_0;
    char cmndname = 0; // current command name
    std::string v = "";
    // write the results to an output iterator
    for (char c : std::regex_replace(command__, command_rex, "")) {
        if (c == ';') break;
        if ((c >= 'a') && (c <= 'z')) c = c + 'A' - 'a';
        if ((c >= 'A') && (c <= 'Z')) {
            cmndname = c;
            v = "";
            ret_0[cmndname] = v;
        } else {
            v = v + c;
            if (cmndname == 0) throw std::invalid_argument("gcode line cannot start with number");
            ret_0[cmndname] = v;//std::stod(v);
        }
    }
    std::map<char, double> ret;
    for(auto it = ret_0.begin(); it != ret_0.end(); ) {
        size_t _idx = 0;
        ret[it->first] = std::stod(it->second, &_idx);
        if (_idx < it->second.size()) throw std::invalid_argument("this is not a number");
        it = ret_0.erase(it);
    }
    return ret;
}

} // namespace gcd
} // namespace raspigcd
