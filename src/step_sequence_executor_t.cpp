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

#include <step_sequence_executor_t.hpp>

#include <executor_pi_t.hpp>
#include <executor_sim_t.hpp>

namespace raspigcd {

step_sequence_executor_t& step_sequence_executor_t::get(configuration_t& cfg_)
{
    static step_sequence_executor_t* instance;
    try {
        if (cfg_.simulate_execution) {
            std::cerr << "simulate execution.." << std::endl;
            executor_sim_t* inst = &(executor_sim_t::get(cfg_));
            instance = inst;
            inst->debug_callback = [](double T,
                                       distance_t target_position,
                                       distance_t position,
                                       distance_t velocity,
                                       distance_t force,
                                       distance_t friction,
                                       int num_empty_ticks) {
                std::cout << "sim" << T << " " << (target_position[0]) << " " << (target_position[1]) << " " << (target_position[2]) << " " << (target_position[3]) << " " << (position[0]) << " " << (position[1]) << " " << (position[2]) << " " << (position[3]) << " " << (velocity[0]) << " " << (velocity[1]) << " " << (velocity[2]) << " " << (velocity[3]) << " " << (force[0]) << " " << (force[1]) << " " << (force[2]) << " " << (force[3]) << " " << (friction[0]) << " " << (friction[1]) << " " << (friction[2]) << " " << (friction[3]) << " " << num_empty_ticks << "\n";
            };
        } else {
            instance = &(executor_pi_t::get(cfg_));
        }

    } catch (...) {
        instance = &(executor_sim_t::get(cfg_));
    }
    return *instance;
}

steps_t step_sequence_executor_t::commands_to_steps(const std::vector<executor_command_t>& commands)
{
    steps_t ret(0, 0, 0, 0);
    for (const auto& c : commands) {
        int r = c.cmnd.repeat;
        do {
            int dir[4] = {0, 0, 0, 0};
            // step direction
            for (auto i : {0, 1, 2, 3})
                dir[i] = c.b[i].step * (c.b[i].dir * 2 - 1);
            for (int i : {0, 1, 2, 3}) {
                ret[i] += dir[i];
            }
        } while ((r--) > 0);
    }
    return ret;
}

} // namespace raspigcd
