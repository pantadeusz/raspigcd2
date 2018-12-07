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

#ifndef __RASPIGCD_GCD_PATH_INTENT_EXECUTION_HPP__
#define __RASPIGCD_GCD_PATH_INTENT_EXECUTION_HPP__

#include <configuration.hpp>
#include <gcd/gcode_interpreter_objects_t.hpp>
#include <hardware/motor_layout.hpp>
#include <hardware/low_steppers.hpp>
#include <hardware/stepping.hpp>
#include <movement/path_intent_t.hpp>
#include <movement/steps_analyzer.hpp>
#include <movement/steps_generator.hpp>
#include <movement/variable_speed.hpp>

#include <chrono> // std::chrono::seconds
#include <future>
#include <map>
#include <memory>
#include <thread> // std::this_thread::sleep_for
#include <mutex>
#include <future>


#include <iostream>

namespace raspigcd {
namespace gcd {

struct path_intent_executor_result_t {
    std::vector<std::string> errors;
    steps_t position;
};

class path_intent_executor
{
    std::mutex _execute_mutex;    
    gcode_interpreter_objects_t _gcdobjs;


public:
    /**
     * @brief execute path intent
     * 
     * path intent is a list of node points and transitions that make the whole execution plan.
     * 
     * The method executes the path intent. This method will run exclusively, so no other execute can be executed at the same time.
     * 
     */
    path_intent_executor_result_t execute(const movement::path_intent_t& path_intent);


    /**
     * @brief Terminate currently executing path intent. The execute command shall be stopped
     * 
     */
    void terminate();

    /**
     * @brief Executes pure path intent, that is with only positions and moves
     */
    //void execute_pure_path_intent(const movement::path_intent_t& path_intent);

    void set_gcode_interpreter_objects(const gcode_interpreter_objects_t &gcdobjs_) {
        std::lock_guard<std::mutex> guard(_execute_mutex);
        if (gcdobjs_.stepping.get() == nullptr) throw std::invalid_argument("steping must be set for  set_gcode_interpreter_objects");
        _gcdobjs = gcdobjs_;
    }
    gcode_interpreter_objects_t get_gcode_interpreter_objects() const {
        return _gcdobjs;
    }
}; // namespace gcd


} // namespace gcd
} // namespace raspigcd

#endif

/*


async experiments


path_intent_executor_result_t path_intent_executor::execute(const movement::path_intent_t& path_intent)
{
    std::lock_guard<std::mutex> guard(_execute_mutex);
    movement::path_intent_t path_fragment_to_go{};

    movement::steps_generator steps_generator_drv(_gcdobjs.motor_layout);
    movement::variable_speed variable_speed_driver(_gcdobjs.motor_layout, _gcdobjs.configuration, _gcdobjs.configuration.tick_duration());

    std::list<std::function<void()>> actions_list;
    std::vector<std::future<hardware::multistep_commands_t>> actions_futures;
    actions_futures.reserve(1024);
    std::mutex mpe_mutex;
    int mpe_calculated_size = 0;

    for (const auto& element : path_intent) {
        if (element.index() < 2) {
            path_fragment_to_go.push_back(element);
        } else {
            if (path_fragment_to_go.size() > 0) {
                actions_futures.push_back(std::async(
                    std::launch::async,
                    [this, &mpe_mutex, &steps_generator_drv, &variable_speed_driver, &mpe_calculated_size](const movement::path_intent_t& path_fragment_to_go) -> hardware::multistep_commands_t {
                        std::lock_guard<std::mutex> lock(mpe_mutex);
                        auto plan_to_execute = variable_speed_driver.intent_to_movement_plan(path_fragment_to_go);
                        hardware::multistep_commands_t ticks_to_execute;
                        steps_generator_drv.movement_plan_to_step_commands(plan_to_execute, _gcdobjs.configuration.tick_duration(),
                            [&, this](std::unique_ptr<hardware::multistep_commands_t> msteps_p) {
                                ticks_to_execute.insert(ticks_to_execute.end(), msteps_p.get()->begin(), msteps_p.get()->end());
                            });
                        return ticks_to_execute;
                    },
                    path_fragment_to_go));
                actions_list.push_back([this, &actions_futures, i = (actions_futures.size() - 1)]() { _gcdobjs.stepping.get()->exec(actions_futures.at(i).get()); });
                path_fragment_to_go.clear();
            }
            switch (element.index()) {
            case 2: //path_intentions::pause_t
                actions_list.push_back([this, element]() { _gcdobjs.timers.get()->wait_s(std::get<movement::path_intentions::pause_t>(element).delay_s); });
                break;
            case 3: // path_intentions::spindle_t
                actions_list.push_back([this, element]() {
                    for (const auto& s : std::get<movement::path_intentions::spindle_t>(element).spindle) {
                        _gcdobjs.spindles_pwm.get()->spindle_pwm_power(s.first, s.second);
                    }
                    _gcdobjs.timers.get()->wait_s(std::get<movement::path_intentions::spindle_t>(element).delay_s);
                });
                break;
            case 4: // path_intentions::motor_t
                actions_list.push_back([this, element]() {
                    _gcdobjs.steppers.get()->enable_steppers(std::get<movement::path_intentions::motor_t>(element).motor);
                    _gcdobjs.timers.get()->wait_s(std::get<movement::path_intentions::motor_t>(element).delay_s);
                });
                break;
            default:
                throw std::invalid_argument("path_intent_executor::execute: unsupported element");
            }
        }
    }
    if (path_fragment_to_go.size() > 0) {
        actions_futures.push_back(std::async(
            std::launch::async,
            [this, &mpe_mutex, &steps_generator_drv, &variable_speed_driver, &mpe_calculated_size](const movement::path_intent_t& path_fragment_to_go) -> hardware::multistep_commands_t {
                std::lock_guard<std::mutex> lock(mpe_mutex);
                auto plan_to_execute = variable_speed_driver.intent_to_movement_plan(path_fragment_to_go);
                hardware::multistep_commands_t ticks_to_execute;
                steps_generator_drv.movement_plan_to_step_commands(plan_to_execute, _gcdobjs.configuration.tick_duration(),
                    [&, this](std::unique_ptr<hardware::multistep_commands_t> msteps_p) {
                        ticks_to_execute.insert(ticks_to_execute.end(), msteps_p.get()->begin(), msteps_p.get()->end());
                    });
                return ticks_to_execute;
            },
            path_fragment_to_go));
        actions_list.push_back([this, &actions_futures, i = (actions_futures.size() - 1)]() { _gcdobjs.stepping.get()->exec(actions_futures.at(i).get()); });
        path_fragment_to_go.clear();
    }
    for (auto& e : actions_list) {
        e();
    }
    return {};
}










*/