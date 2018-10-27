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


#include <movement_simple_steps.hpp>
#include <distance_t.hpp>
#include <hardware_stepping_commands.hpp>
#include <list>
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

/**
 * @brief generates steps to reach given destination steps
 * @arg steps_ current steps count
 * @arg destination_steps_ desired steps count
 */
std::vector<hardware::multistep_command> chase_steps(const steps_t& start_pos_, steps_t destination_pos_)
{
    std::vector<hardware::multistep_command> ret;
    ret.reserve(steps_remaining(start_pos_, destination_pos_) * 2 + 32);
    auto steps = start_pos_;
    hardware::multistep_command executor_command;
    do {
        executor_command.v = 0;
        executor_command.cmnd.count = 1;
        for (unsigned int i = 0; i < steps.size(); i++) {
            executor_command.cmnd.b[i].dir = ((destination_pos_[i] > steps[i]) ? 1 : 0);
            executor_command.cmnd.b[i].step = (destination_pos_[i] - steps[i]) ? 1 : 0;
            if (destination_pos_[i] > steps[i])
                steps[i]++;
            else if (destination_pos_[i] < steps[i])
                steps[i]--;
        }
        ret.push_back(executor_command);
    } while (steps_remaining(steps, destination_pos_) > 0);
    return ret;
}


} // namespace simple_steps

} // namespace movement
} // namespace raspigcd
