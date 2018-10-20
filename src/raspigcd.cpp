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

#include <g_code_interpreter_t.hpp>

#include <chrono>
#include <configuration_t_json.hpp>
#include <distance_t.hpp>
#include <fstream>
#include <future>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <motion_plan_t.hpp>
#include <motor_layout_t.hpp>
#include <mutex>
#include <regex>
#include <set>
#include <sstream>
#include <step_sequence_executor_t.hpp>
#include <streambuf>
#include <string>
#include <thread>
#include <tuple>
#include <vector>


using namespace raspigcd;

/**
 * @brief generates sinusoidal wave with given maximal amplitude in milimeters and given time in seconds
 * 
 */
std::vector<executor_command_t> generate_sin_wave_for_test(
    configuration_t* _cfg,
    double amplitude = 15, //< in milimeters
    double T = 10,         //< in seconds
    int axis = 2           //< axis to move
    )
{
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

void help_screen(const std::vector<std::string>& args)
{
    std::cout << "raspigcd2 - tool for executing gcode on Raspberry Pi" << std::endl;
    std::cout << "Usage:" << std::endl;
    std::cout << " $ " << args[0] << "[arguments1] [argument2] ..." << std::endl;
    std::cout << R"(Arguments can be:
    -f [filename]   - opens given file with gcode text
    -c [conffile]   - reads configuration from given file. Configuration should be in json format
    -C [conffile]   - saves configuration to given file
    )" << std::endl;
}

int main(int argc, char** argv)
{
    std::vector<std::string> args(argv, argv + argc);
    std::map<std::string, std::string> args_p;
    if (args.size() == 1) {
        help_screen(args);
        return -1;
    }
    for (unsigned int i = 1; i < args.size() - 1; i++) {
        if ((args[i].size() > 0) && (args[i][0] == '-')) {
            args_p[args[i]] = args[i + 1];
        }
    }

    //    static auto& cfg = configuration_t::get().load_defaults();
    auto cfg = configuration_t().load_defaults();
    //std::cout << cfg << std::endl;
    if (args_p.count("-c")) cfg.load(args_p["-c"]);
    if (args_p.count("-C")) cfg.save(args_p["-C"]);
    if (args_p.count("-f")) {
        step_sequence_executor_t& executor = step_sequence_executor_t::get(cfg);

        executor.enable(true);
        //executor.execute(generate_sin_wave_for_test());
        /*    {
        motion_plan_t mp(cfg);
        mp.gotoxy(distance_t{15.0, 0.0, 0.0, 0.0}, 160.0)
            .gotoxy(distance_t{15.0, -2.0, 0.0, 0.0}, 20.0)
            .gotoxy(distance_t{15.0, -15.0, 0.0, 0.0}, 5.0)
            .gotoxy(distance_t{0.0, -15.0, 0.0, 0.0}, 20.0)
            .gotoxy(distance_t{0.0, 0.0, 0.0, 0.0}, 20.0);
        //.gotoxy(distance_t{3000.0, 0.0, 0.0, 0.0}, 20.0);
        executor.execute(mp.get_motion_plan());
    }
    if (0) {
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
*/

        {
            buttons_t btns(&cfg);
            motion_plan_t mp(cfg);
            auto spindles = generic_spindle_t::get(cfg);
            g_code_interpreter_t gcdinterpert(&cfg, &executor, spindles, &mp);

            std::ifstream gcode_file_stream(args_p["-f"]);
            std::string gcode_string((std::istreambuf_iterator<char>(gcode_file_stream)),
                std::istreambuf_iterator<char>());

            auto job_task = std::async(std::launch::async,
                [&gcode_string, &gcdinterpert]() {
                    return gcdinterpert.execute_gcode_string(gcode_string);
                });
            while (job_task.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready) {
                std::cout << "working and waiting..." << std::endl;
            }

            std::list<std::string> ret_lines = job_task.get();


            /*        gcdinterpert.execute_gcode_string(R"GCODE(
; jan
M3P1000
M17
M3
; nowag
G0Z5
G0X-10Y-20
M114
G1X0Y-20
G1X0Y0
M114
G0X0Y0Z0
M5
M18
)GCODE");
*/
        }

        executor.enable(false);
    }
    return 0;
}
