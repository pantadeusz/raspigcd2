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
#include <movement/physics.hpp>
#include <movement/simple_steps.hpp>

#include <functional>

namespace raspigcd {
namespace converters {

raspigcd::hardware::multistep_commands_t __generate_g1_steps(
    const raspigcd::gcd::block_t &state,
    const raspigcd::gcd::block_t &next_state,
    double dt,
    hardware::motor_layout& ml_
    ) {
    using namespace raspigcd::hardware;
    using namespace raspigcd::gcd;
    using namespace raspigcd::movement::simple_steps;
    using namespace movement::physics;
    auto pos_from = gcd::block_to_distance_t(state);
    auto pos_to = gcd::block_to_distance_t(next_state);

    double l = (pos_to - pos_from).length(); // distance to travel
    double v0 = state.at('F');                  // velocity
    double v1 = next_state.at('F');             // velocity
    std::list<multistep_command> fragment;   // fraagment of the commands list generated in this stage
    steps_t final_steps;                     // steps after the move
    if (l > 0) {
        if ((v0 == v1)) {
            if (v1 == 0) throw std::invalid_argument("the feedrate should not be 0 for non zero distance");
            auto pos = pos_from;
            double s = v1 * dt; // distance to go
            auto direction = (pos_to - pos_from) / l;
            auto pos_from_steps = ml_.cartesian_to_steps(pos); //configuration(state);
            for (int i = 1; s <= l; ++i, s = v1 * (dt * i)) {
                // TODO: Create test case for this situation!!!!
                auto np = pos_from + direction * s;
                auto pos_to_steps = ml_.cartesian_to_steps(np); //gcd::block_to_distance_t(next_state);
                multistep_commands_t steps_todo = chase_steps(pos_from_steps, pos_to_steps);
                fragment.insert(fragment.end(), steps_todo.begin(), steps_todo.end());
                pos = np;
                pos_from_steps = pos_to_steps;
            }
            final_steps = pos_from_steps;
        } else if ((v1 != v0)) {
            auto direction = (pos_to - pos_from) / l;
            const path_node_t pn_a{.p = pos_from, .v = v0};
            const path_node_t pn_b{.p = pos_to, .v = v1};
            double a = acceleration_between(pn_a, pn_b);
            //std::cout << "a = " << a << std::endl;
            double t = dt;                                       ///< current time
            auto l = [&]() { return v0 * t + 0.5 * a * t * t; }; ///< current distance from p0
            double s = (pos_to - pos_from).length();             // distance to travel
            auto p_steps = ml_.cartesian_to_steps(pos_from);
            for (int i = 1; l() < s; ++i, t = dt * i) {
                auto pos = ml_.cartesian_to_steps(pos_from + direction * l());
                multistep_commands_t steps_to_add = chase_steps(p_steps, pos);
                fragment.insert(fragment.end(), steps_to_add.begin(), steps_to_add.end());
                p_steps = pos;
            }
            final_steps = p_steps;
        }
        auto pos_to_steps = ml_.cartesian_to_steps(pos_to);
        if (!(final_steps == pos_to_steps)) { // fix missing steps
            multistep_commands_t steps_todo = chase_steps(final_steps, pos_to_steps);
            fragment.insert(fragment.end(), steps_todo.begin(), steps_todo.end());
        }
        auto collapsed = collapse_repeated_steps(fragment);
        return collapsed;
    }
    return {};
}

hardware::multistep_commands_t program_to_steps(
    const gcd::program_t& prog_,
    const configuration::actuators_organization& conf_,
    hardware::motor_layout& ml_,
    const gcd::block_t& initial_state_,
    std::function<void(const gcd::block_t &)> finish_callback_f_)
{
    using namespace raspigcd::hardware;
    using namespace raspigcd::gcd;
    using namespace raspigcd::movement::simple_steps;
    using namespace movement::physics;
    auto state = initial_state_;
    std::list<multistep_command> result;
    //double dt = 0.000001 * (double)conf_.tick_duration_us;//
    double dt = ((double)conf_.tick_duration_us) / 1000000.0;
    // std::cout << "dt = " << dt << std::endl;
    for (const auto& block : prog_) {
        auto next_state = gcd::merge_blocks(state, block);

        if (next_state.at('G') == 92) {
            // change position, but not generate steps
        } else if (next_state.at('G') == 4) {
            double t = 0;
            if (next_state.count('X')) { // seconds
                t = next_state.at('X');
            } else if (next_state.count('P')) {
                t = next_state.at('P')/ 1000.0;
            }
            hardware::multistep_command executor_command = {};
            executor_command.count = t/dt;
            result.push_back(executor_command);
            next_state = state;
        } else if ((next_state.at('G') == 1) || (next_state.at('G') == 0)) {
            std::cout << "STATE: ";
            for (char idx : {'X','Y','Z'}) {
                std::cout << idx << state[idx] << " ";
            }
            std::cout << "  -->  ";
            for (char idx : {'X','Y','Z'}) {
                std::cout << idx << next_state[idx] << " ";
            }
            std::cout << std::endl;
            auto collapsed = __generate_g1_steps( state, next_state, dt, ml_ );
            result.insert(result.end(), collapsed.begin(), collapsed.end());
        }
        state = next_state;
        finish_callback_f_(state);
    }
    return collapse_repeated_steps(result);
}

} // namespace converters
} // namespace raspigcd
