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


#include <converters/gcd_program_to_steps.hpp>
#include <movement/simple_steps.hpp>

namespace raspigcd {
namespace converters {

hardware::multistep_commands_t program_to_steps(
    const gcd::program_t& prog_,
    const configuration::actuators_organization& conf_,
    hardware::motor_layout &ml_,
    const gcd::block_t& initial_state_)
{
    using namespace raspigcd::hardware;
    using namespace raspigcd::gcd;
    using namespace raspigcd::movement::simple_steps;
    auto state = initial_state_;
    multistep_commands_t result;
    //double dt = 0.000001 * (double)conf_.tick_duration_us;//
    double dt = ((double) conf_.tick_duration_us)/1000000.0;
    std::cout << "dt = " << dt << std::endl;
    for (const auto &block: prog_) {
        auto next_state = gcd::merge_blocks(state, block);
        auto pos_from = gcd::block_to_distance_t(state);
        auto pos_to = gcd::block_to_distance_t(next_state);

        double l = (pos_to-pos_from).length();// distance to travel 
        double v = next_state['F']; // velocity
        if (l > 0) {
            auto pos = pos_from;
            double s = v*dt; // distance to go
            auto direction = (pos_to-pos_from)/l;
            auto pos_from_steps = ml_.cartesian_to_steps(pos);//configuration(state);
            for (int i = 1; s <= l; ++i,s = v*(dt*i)) {
                auto np = direction * s;
                auto pos_to_steps = ml_.cartesian_to_steps(np);//gcd::block_to_distance_t(next_state);
                multistep_commands_t steps_todo = chase_steps(pos_from_steps, pos_to_steps);
                result.insert(result.end(),steps_todo.begin(),steps_todo.end());
                pos = np;
                pos_from_steps = pos_to_steps;      
            }
            auto pos_to_steps = ml_.cartesian_to_steps(pos_to);
            if (!(pos_from_steps == pos_to_steps)){
                multistep_commands_t steps_todo = chase_steps(pos_from_steps, pos_to_steps);
                result.insert(result.end(),steps_todo.begin(),steps_todo.end());
            }
        }
        state = next_state;
    }
    return result;
}

} // namespace converters
} // namespace raspigcd
