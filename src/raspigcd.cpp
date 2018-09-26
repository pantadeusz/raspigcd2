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

#include <chrono>
#include <configuration_t_json.hpp>
#include <motion_plan_t.hpp>
#include <distance_t.hpp>
#include <executor_t.hpp>
#include <iostream>
#include <list>
#include <motor_layout_t.hpp>
#include <string>
#include <tuple>
#include <vector>
#include <memory>

using namespace raspigcd;

/**
 * @brief generates sinusoidal wave with given maximal amplitude in milimeters and given time in seconds
 * 
 */
std::vector<executor_command_t> generate_sin_wave_for_test(double amplitude = 15, //< in milimeters
    double T = 10,                                                                //< in seconds
    int axis = 2                                                                  //< axis to move
)
{
    auto _cfg = &(configuration_t::get());

    std::vector<executor_command_t> executor_commands;
    executor_commands.reserve((::size_t)(T / _cfg->tick_duration));
    steps_t steps;

    for (int i = 0; i < 100; i++) {
        executor_command_t e;
        e.v = 0;
        executor_commands.push_back(e);
    }

    //    steps[axis] = _cfg->hardware.steppers[axis].stepsPerMm*std::cos(0)*amplitude;
    int minimal_step_skip = 1000000000;
    for (double t = 0.0; t < T; t += _cfg->tick_duration) {
        steps_t new_steps;
        new_steps[axis] = _cfg->hardware.steppers[axis].steps_per_mm * std::cos(t * 3.141592653589793238462643 * 5) * amplitude * std::sin(3.141592653589793238462643 * t / T);
        auto st = motion_plan_t::chase_steps(steps, new_steps);
        if (st.size() == 1) {
            if (st.back().v != 0) {
                int d = 0;
                for (int i = executor_commands.size() - 1; i > 0; i--) {
                    if (executor_commands[i].v != 0)
                        break;
                    else
                        d++;
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



int main(int argc, char** argv)
{
    std::vector<std::string> args(argv, argv + argc);

    static auto& cfg = configuration_t::get().load_defaults();
    std::cout << cfg << std::endl;
    executor_t& executor = executor_t::get(cfg);

    executor.enable(true);
    //executor.execute(generate_sin_wave_for_test());
    {
        motion_plan_t mp(cfg);
        mp.gotoxy(distance_t{15.0, 0.0, 0.0, 0.0}, 20.0)
            .gotoxy(distance_t{15.0, -2.0, 0.0, 0.0}, 20.0)
            .gotoxy(distance_t{15.0, -15.0, 0.0, 0.0}, 5.0)
            .gotoxy(distance_t{0.0, -15.0, 0.0, 0.0}, 20.0)
            .gotoxy(distance_t{0.0, 0.0, 0.0, 0.0}, 100.0);
            //.gotoxy(distance_t{3000.0, 0.0, 0.0, 0.0}, 20.0);
        executor.execute(mp.get_motion_plan());
    }
    if(0) {
        motion_plan_t mp(cfg);
        mp.gotoxy(distance_t{5.0, 0.0, 0.0, 0.0}, 20.0);
        executor.execute(mp.get_motion_plan());
        mp.clear_motion_fragments_buffer();

        mp.gotoxy(distance_t{5.0, -2.0, 0.0, 0.0}, 20.0);
        executor.execute(mp.get_motion_plan());
        mp.clear_motion_fragments_buffer();

        mp.gotoxy(distance_t{5.0, -5.0, 0.0, 0.0}, 5.0);
        executor.execute(mp.get_motion_plan());
        mp.clear_motion_fragments_buffer();

        mp.gotoxy(distance_t{0.0, -5.0, 0.0, 0.0}, 20.0);
        executor.execute(mp.get_motion_plan());
        mp.clear_motion_fragments_buffer();

        mp.gotoxy(distance_t{0.0, 0.0, 0.0, 0.0}, 20.0);
        executor.execute(mp.get_motion_plan());
    }
    executor.enable(false);

    return 0;
}
