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

/* movement::path_intent_t generate_path_intent(const program_t &parsed_program_) {
    // todo..
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
} */


program_t apply_limits_for_turns (const program_t& program_states,
                const configuration::limits &machine_limits) {
    auto ret_states = program_states;
    if (ret_states.size() == 0) { return {}; }
    if (ret_states.size() == 1) {
        auto &state = ret_states[0];
        if ((state.count('F') > 0) && (state.at('F') > 0)) {
            state['F'] = (
            machine_limits.max_no_accel_velocity_mm_s[0]+
            machine_limits.max_no_accel_velocity_mm_s[1]+
            machine_limits.max_no_accel_velocity_mm_s[2]+
            machine_limits.max_no_accel_velocity_mm_s[3])/4;
        }
        return ret_states;
    }
    // first block, and last block
    //auto first_diff = ret_states[1] - ret_states[0];
    //block_t previous_block;
    // for (::size_t i = 1; i < ret_states.size(); i++) {
    //     if (previous_block.size() == 0) {

    //     } else {

    //     }
    //     previous_block = state;
    // }
    return ret_states;
}


std::string back_to_gcode(partitioned_program_t &btg) {
    std::stringstream strs;
    for (auto &group : btg) {
        strs << "; Group of size " << group.size() << std::endl;
        for (auto &block : group) {
            for (auto &e: block) {
                strs << "" << e.first << e.second << " ";
            }
            strs << std::endl;
        }
    }
    return strs.str();
}

block_t merge_blocks(const block_t &destination, const block_t &source) {
    block_t merged = destination;
    for (const auto &sb : source) {
        merged[sb.first] = sb.second;
    }
    return merged;
}

partitioned_program_t group_gcode_commands(const program_t& program_states, const block_t & initial_state ) {
    partitioned_program_t generated_program;
    block_t current_state = merge_blocks({{'X',0},{'Y',0},{'Z',0}},initial_state);
    for (const auto &e: program_states) {
        if (generated_program.size() == 0) {
            if ((e.count('G') > 0) || (e.count('M'))) generated_program.push_back({e});
            else throw std::invalid_argument("the first command must be G or M");
        } else {
            if (e.count('G')) {
                if (generated_program.back().back().count('G')) {
                    if (generated_program.back().back().at('G') == e.at('G')) {
                        generated_program.back().push_back(e);
                    } else {
                        generated_program.push_back({e});
                    }
                } else {
                    generated_program.push_back({e});
                }
            } else if (e.count('M')){
                    generated_program.push_back({e});
            } else {
                if (generated_program.back().front().count('G')) generated_program.back().push_back(e);
                else throw std::invalid_argument("It is not clear if the command is about G or M");
            }
        }
        current_state = merge_blocks(current_state,e);
        /// check correctness of first command
        if((generated_program.back().size() == 1) 
            && (generated_program.back()[0].count('G') == 1)
            ){
            if ((((int)generated_program.back()[0].at('G')) == 1) &&
                (((int)generated_program.back()[0].count('F')) == 0)) {
        //        // we must provide feedrate for the first command
                generated_program.back() = {{{'G',1.0},{'F',current_state.at('F')}},e};
            }
        }
    }


    return generated_program;
}


program_t gcode_to_maps_of_arguments(const std::string &program_) {
    // command_to_map_of_arguments
    program_t ret;
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
