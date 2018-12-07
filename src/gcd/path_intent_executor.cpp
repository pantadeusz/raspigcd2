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

#include <configuration.hpp>
#include <gcd/gcode_interpreter_objects_t.hpp>
#include <gcd/path_intent_executor.hpp>

#include <chrono> // std::chrono::seconds
#include <future>
#include <map>
#include <memory>
#include <thread> // std::this_thread::sleep_for

namespace raspigcd {
namespace gcd {


void path_intent_executor::terminate()
{
    _gcdobjs.stepping.get()->terminate();
}

path_intent_executor_result_t path_intent_executor::execute(const movement::path_intent_t& path_intent)
{
    std::lock_guard<std::mutex> guard(_execute_mutex);

    movement::steps_generator steps_generator_drv(_gcdobjs.motor_layout);
    movement::variable_speed variable_speed_driver(_gcdobjs.motor_layout, _gcdobjs.configuration, _gcdobjs.configuration.tick_duration());

    // this method works asynchronous - it calculates steps in another thread that feeds the results to the main thread. This allows for no breaking execution of code

    path_intent_executor_result_t return_val = {.errors={},.position = {0, 0, 0, 0}};

    // group actions to form movement fragments and actions
    std::vector<movement::path_intent_t> path_fragments; // fragments to execute in sequence
    for (const auto& element : path_intent) {
        if ((path_fragments.size() == 0) || (element.index() >= 2)) {
            path_fragments.push_back({element});
        } else {
            if (path_fragments.back().front().index() == 0) {
                path_fragments.back().push_back(element);
            } else {
                path_fragments.push_back({element});
            }
        }
    }

    // build executor actions list
    std::vector<std::future<hardware::multistep_commands_t>> actions_futures;
    actions_futures.reserve(1024);
    std::mutex mpe_mutex;
    int mpe_calculated_size = 0;
    std::list<std::function<void()>> actions_list;

    for (const auto& path_fragment_to_go : path_fragments) {
        switch (path_fragment_to_go.front().index()) {
        case 0:
            actions_futures.push_back(std::async(
                std::launch::async,
                [this, &mpe_mutex, &steps_generator_drv, &variable_speed_driver, &mpe_calculated_size](const movement::path_intent_t& path_fragment_to_go) -> hardware::multistep_commands_t {
                    std::lock_guard<std::mutex> lock(mpe_mutex);
                    // only movement commands
                    auto plan_to_execute = variable_speed_driver.intent_to_movement_plan(path_fragment_to_go);
                    hardware::multistep_commands_t ticks_to_execute;
                    steps_generator_drv.movement_plan_to_step_commands(plan_to_execute, _gcdobjs.configuration.tick_duration(),
                        [&, this](std::unique_ptr<hardware::multistep_commands_t> msteps_p) {
                            ticks_to_execute.insert(ticks_to_execute.end(), msteps_p.get()->begin(), msteps_p.get()->end());
                        });
                    return ticks_to_execute;
                },
                path_fragment_to_go));
            actions_list.push_back([this, &return_val, pos_after = path_fragment_to_go.back(), &actions_futures, i = (actions_futures.size() - 1)]() {
                auto commands_to_do = actions_futures.at(i).get(); // get the ticks that should be executed. This is generated in future
                try {
                    _gcdobjs.stepping.get()->exec(commands_to_do);
                } catch (const raspigcd::hardware::execution_terminated& e) {
                    // update positions...
                    raspigcd::hardware::execution_terminated e_ = e;
                    return_val.errors.push_back(e.what());

                    std::list<steps_t> steps_from_commands_ = hardware_commands_to_steps(commands_to_do);
                    std::vector<steps_t> steps_from_commands(steps_from_commands_.begin(),steps_from_commands_.end());
                    
                    return_val.position = return_val.position + (steps_from_commands.at(_gcdobjs.stepping.get()->get_tick_index()-1));
                    throw e_;
                }
                return_val.position = _gcdobjs.motor_layout.get()->cartesian_to_steps(std::get<distance_t>(pos_after));
            });

            break;
        case 2: //path_intentions::pause_t
            actions_list.push_back([this, path_fragment_to_go]() {
                _gcdobjs.timers.get()->wait_s(std::get<movement::path_intentions::pause_t>(path_fragment_to_go.front()).delay_s);
            });
            break;
        case 3: // path_intentions::spindle_t
            actions_list.push_back([this, path_fragment_to_go]() {
                for (const auto& s : std::get<movement::path_intentions::spindle_t>(path_fragment_to_go.front()).spindle) {
                    _gcdobjs.spindles_pwm.get()->spindle_pwm_power(s.first, s.second);
                }
                _gcdobjs.timers.get()->wait_s(std::get<movement::path_intentions::spindle_t>(path_fragment_to_go.front()).delay_s);
            });
            break;
        case 4: // path_intentions::motor_t
            actions_list.push_back([this, path_fragment_to_go]() {
                _gcdobjs.steppers.get()->enable_steppers(std::get<movement::path_intentions::motor_t>(path_fragment_to_go.front()).motor);
                _gcdobjs.timers.get()->wait_s(std::get<movement::path_intentions::motor_t>(path_fragment_to_go.front()).delay_s);
            });
            break;
        default:
            throw std::invalid_argument("path_intent_executor::execute: unsupported element");
        }
    }

    // exec actions list
    for (auto& e : actions_list) {
        try {
            e();
        } catch (const raspigcd::hardware::execution_terminated& e) {
            _gcdobjs.stepping.get()->reset_after_terminate();
            return return_val;
        }
    }
    return return_val;
}


void path_intent_executor::execute_pure_path_intent(const movement::path_intent_t& path_intent)
{
    movement::steps_generator steps_generator_drv(_gcdobjs.motor_layout);
    movement::variable_speed variable_speed_driver(_gcdobjs.motor_layout, _gcdobjs.configuration, _gcdobjs.configuration.tick_duration());
    //movement::steps_analyzer sa(_gcdobjs.motor_layout);

    for (const auto& e : path_intent) {
        if (e.index() > 1) throw std::invalid_argument("only position and move are allowed in execute_pure_path_intent");
    }
    auto plan_to_execute = variable_speed_driver.intent_to_movement_plan(path_intent);

    // TODO: optimizations
    hardware::multistep_commands_t ticks_to_execute;

    steps_generator_drv.movement_plan_to_step_commands(plan_to_execute, _gcdobjs.configuration.tick_duration(),
        [&, this](std::unique_ptr<hardware::multistep_commands_t> msteps_p) {
            // execute now or later?
            ticks_to_execute.insert(ticks_to_execute.end(), msteps_p.get()->begin(), msteps_p.get()->end());
        });
    auto handle = std::async(
        std::launch::async,
        [this](const hardware::multistep_commands_t& ttr) {
            _gcdobjs.stepping.get()->exec(ttr);
        },
        ticks_to_execute);
    //_gcdobjs.stepping.get()->exec(ticks_to_execute);

    // auto ticks_difference = sa.steps_from_tick(ticks_to_execute, sa.get_last_tick_index(ticks_to_execute));
    // {
    //     psteps = psteps + ticks_difference;
    // }

    handle.get();
}

#ifdef NIEZADEFINIOWANE

void path_intent_executor::execute(const movement::path_intent_t& path_intent)
{
    std::lock_guard<std::mutex> guard(_execute_mutex);
    movement::steps_generator steps_generator_drv(_gcdobjs.motor_layout);
    movement::variable_speed variable_speed_driver(_gcdobjs.motor_layout, _gcdobjs.configuration, _gcdobjs.configuration.tick_duration());
    movement::steps_analyzer sa(_gcdobjs.motor_layout);

    // _gcdobjs.hardware_driver.get()->enable_steppers({true});
    movement::path_intent_t program_movement_to_execute_at_once = {};

    steps_t psteps = {0, 0, 0, 0};

    auto execute_program_movement = [this, &psteps, &sa, &steps_generator_drv, &variable_speed_driver](movement::path_intent_t program_) {
        /**
             * first, convert intentions into plan to execute (plan_to_execute)
             * plan_to_execute should be optimized for best movements (accelerations, breaks, corners optimization and so on)
             * then generate steps for each fragment and collect them to have one full movements fragment.
             * and last execute everything at once using stepping object.
             */
        auto plan_to_execute = variable_speed_driver.intent_to_movement_plan(program_);
        hardware::multistep_commands_t ticks_to_execute;
        steps_generator_drv.movement_plan_to_step_commands(plan_to_execute, _gcdobjs.configuration.tick_duration(),
            [&, this](std::unique_ptr<hardware::multistep_commands_t> msteps_p) {
                ticks_to_execute.insert(ticks_to_execute.end(), msteps_p.get()->begin(), msteps_p.get()->end());
                //std::cout << "steps to execute: " << msteps_p.get()->size() << std::endl;
            });
        auto handle = std::async(std::launch::async, [this](const hardware::multistep_commands_t& ttr) { _gcdobjs.stepping.get()->exec(ttr); }, ticks_to_execute);
        //_gcdobjs.stepping.get()->exec(ticks_to_execute);
        psteps = psteps + sa.steps_from_tick(ticks_to_execute, sa.get_last_tick_index(ticks_to_execute));
        std::cout << "position is " << psteps << std::endl;
        handle.get();
    };


    for (auto& element : path_intent) {
        switch (element.index()) {
        case 0: /// position
        case 1: /// intended velocity in mm/s
            program_movement_to_execute_at_once.push_back(element);
            break;
        case 2: //path_intentions::pause_t
            execute_program_movement(program_movement_to_execute_at_once);
            program_movement_to_execute_at_once.clear();
            // execute the commands intended

            throw std::invalid_argument("TODO: paust_t");
            std::this_thread::sleep_for(std::chrono::milliseconds((int)(std::get<movement::path_intentions::pause_t>(element).delay_s * 1000.0)));
            break;
        case 3: // path_intentions::spindle_t
            execute_program_movement(program_movement_to_execute_at_once);
            program_movement_to_execute_at_once.clear();
            // execute the commands intended

            throw std::invalid_argument("TODO: spindle_t");
            break;
        case 4: // path_intentions::motor_t
            execute_program_movement(program_movement_to_execute_at_once);
            program_movement_to_execute_at_once.clear();
            // execute the commands intended

            std::this_thread::sleep_for(std::chrono::milliseconds((int)(std::get<movement::path_intentions::motor_t>(element).delay_s * 1000.0)));
            _gcdobjs.steppers.get()->enable_steppers(std::get<movement::path_intentions::motor_t>(element).motor);
            break;
        default:
            throw std::invalid_argument("wrong path_intention");
        }
    }
    //_gcdobjs.hardware_driver.get()->enable_steppers({false});
}


#endif


} // namespace gcd
} // namespace raspigcd
