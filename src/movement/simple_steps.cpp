/*
    Raspberry Pi G-CODE interpreter

    Copyright (C) 2019  Tadeusz Puźniakowski puzniakowski.pl

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/





#include <distance_t.hpp>
#include <hardware/stepping_commands.hpp>
#include <list>
#include <movement/simple_steps.hpp>
#include <steps_t.hpp>

namespace raspigcd {
namespace movement {

namespace simple_steps {

int steps_remaining(const steps_t& steps_, const steps_t& destination_steps_)
{
    int ret = 0;
    for (unsigned int i = 0; i < steps_.size(); i++) {
        ret = std::max(std::abs(steps_[i] - destination_steps_[i]), ret);
    }
    return ret;
}

//bool 

hardware::multistep_commands_t collapse_repeated_steps(
    const std::list<hardware::multistep_command>& ret
    ) 
{
    if (ret.size() == 0) return {};
    hardware::multistep_commands_t ret_vect;
    ret_vect.reserve(ret.size());
    // repeated commands should be one with apropriate count
    for (auto& e : ret) {
        if (e.count > 0) {
            if ((ret_vect.size() == 0) || !(multistep_command_same_command( e, ret_vect.back()))) {
                ret_vect.push_back(e);
            } else {
                if (ret_vect.back().count > 0x0fffffff) {
                    ret_vect.push_back(e);
                } else {
                    ret_vect.back().count+=e.count;
                }
            }
        }
    }
    ret_vect.shrink_to_fit();
    return ret_vect;
}


/**
 * @brief generates steps to reach given destination steps
 * @arg steps_ current steps count
 * @arg destination_steps_ desired steps count
 */
hardware::multistep_commands_t chase_steps(const steps_t& start_pos_, steps_t destination_pos_)
{
    hardware::multistep_commands_t ret;
    ret.reserve(steps_remaining(start_pos_, destination_pos_) * 2 + 32);
    auto steps = start_pos_;
    hardware::multistep_command executor_command = {};
    int stodo = steps_remaining(steps, destination_pos_);
    do {
        // executor_command.v = 0;
        executor_command.count = 1;
        for (unsigned int i = 0; i < steps.size(); i++) {
            executor_command.b[i].dir = ((destination_pos_[i] > steps[i]) ? 1 : 0);
            executor_command.b[i].step = (destination_pos_[i] - steps[i]) ? 1 : 0;
            if (destination_pos_[i] > steps[i])
                steps[i]++;
            else if (destination_pos_[i] < steps[i])
                steps[i]--;
        }
        ret.push_back(executor_command);
    } while ((--stodo) > 0);
    return ret;
}


} // namespace simple_steps

} // namespace movement
} // namespace raspigcd
