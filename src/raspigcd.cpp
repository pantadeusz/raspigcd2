/*
    Raspberry Pi G-CODE interpreter
    Copyright (C) 2018  Tadeusz Puźniakowski puzniakowski.pl
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

#include <executor_t.hpp>
#include <configuration_t_json.hpp>
#include <vector>
#include <string>
#include <iostream>

using namespace raspigcd;
int steps_to_do(const steps_t &steps_, const steps_t &destination_steps_)
{
    int ret = 0;
    for (unsigned int i = 0; i < steps_.size(); i++)
    {
        ret = std::max(std::abs(steps_[i] - destination_steps_[i]), ret);
    }
    return ret;
}

/**
 * @brief generates steps to reach given destination steps
 * @arg steps_ current steps count
 * @arg destination_steps_ desired steps count
 */
std::vector<executor_command_t> chase_steps(const steps_t &steps_, steps_t destination_steps_)
{
    //auto &cfg = configuration_t::get();
    std::vector<executor_command_t> ret;
    auto steps = steps_;
    executor_command_t executor_command;
    do
    {
        executor_command.v = 0;
        for (unsigned int i = 0; i < steps.size(); i++)
        {
            executor_command.b[i].dir = ((destination_steps_[i] > steps[i]) ? 1 : 0);// TODO: (not obviosu) * ((cfg.hardware.steppers[i].direction_reverse) ? -1 : 1);
            executor_command.b[i].step = (destination_steps_[i] - steps[i]) ? 1 : 0;
            if (destination_steps_[i] > steps[i])
                steps[i]++;
            else if (destination_steps_[i] < steps[i])
                steps[i]--;
        }
        ret.push_back(executor_command);
    } while (steps_to_do(steps, destination_steps_) > 0);
    return ret;
}

/**
 * @brief generates sinusoidal wave with given maximal amplitude in milimeters and given time in seconds
 * 
 */
std::vector<executor_command_t> generate_sin_wave_for_test(double amplitude = 15, //< in milimeters
                                           double T = 10,         //< in seconds
                                           int axis = 2           //< axis to move
                                           )
{
    auto &cfg = configuration_t::get();

    std::vector<executor_command_t> executor_commands;
    executor_commands.reserve((::size_t)(T/cfg.tick_duration));
    steps_t steps;

    for (int i = 0; i < 100; i++)
    {
        executor_command_t e;
        e.v = 0;
        executor_commands.push_back(e);
    }

    //    steps[axis] = cfg.hardware.steppers[axis].stepsPerMm*std::cos(0)*amplitude;
    int minimal_step_skip = 1000000000;
    for (double t = 0.0; t < T; t += cfg.tick_duration)
    {
        steps_t new_steps;
        new_steps[axis] = cfg.hardware.steppers[axis].steps_per_mm * std::cos(t * 3.141592653589793238462643 * 5) * amplitude * std::sin(3.141592653589793238462643 * t / T);
        auto st = chase_steps(steps, new_steps);
        if (st.size() == 1) {
            if (st.back().v != 0){
                int d = 0;
                for (int i = executor_commands.size() -1; i > 0; i--) {
                     if (executor_commands[i].v != 0) break;
                     else d++;
                }
                if (d < minimal_step_skip) minimal_step_skip = d;
            }
        }
        executor_commands.insert(executor_commands.end(), st.begin(), st.end());
        steps = new_steps;
    }
    std::cerr << "generated " << executor_commands.size() << "steps; minimal step skip: " << minimal_step_skip << std::endl;
    return executor_commands;
}


int main(int argc, char **argv)
{
    std::vector<std::string> args(argv, argv + argc);

    auto &cfg = configuration_t::get().load_defaults();
    std::cout << cfg << std::endl;
    executor_t &executor = executor_t::get();

    executor.enable(true);
    //executor.execute(executor_commands);
    executor.execute(generate_sin_wave_for_test());
    executor.enable(false);

    return 0;
}