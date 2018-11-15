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
#include <gcd/factory.hpp>
#include <hardware/low_steppers.hpp>
#include <hardware/motor_layout.hpp>
#include <hardware/stepping.hpp>
#include <movement/path_intent_t.hpp>
#include <movement/steps_generator.hpp>
#include <movement/variable_speed.hpp>
#include <movement/steps_analyzer.hpp>

#include <chrono> // std::chrono::seconds
#include <map>
#include <memory>
#include <thread> // std::this_thread::sleep_for

namespace raspigcd {
namespace gcd {


class path_intent_executor
{
public:
    gcode_interpreter_objects _gcdobjs;

    void execute(const movement::path_intent_t& path_intent)
    {
        movement::steps_generator steps_generator_drv(_gcdobjs.motor_layout);
        movement::variable_speed variable_speed_driver(_gcdobjs.motor_layout, _gcdobjs.configuration, _gcdobjs.configuration.tick_duration());
        movement::steps_analyzer sa(_gcdobjs.motor_layout);

        // _gcdobjs.hardware_driver.get()->enable_steppers({true});
        movement::path_intent_t program_movement_to_execute_at_once = {};

//sa.get_last_tick_index(ticks_to_execute)
//			std::list<steps_t> steps_in_order = hardware_commands_to_steps(ticks_to_execute);
//            sa.get_last_tick_index(ticks_to_execute);


        steps_t psteps = {0, 0, 0, 0};

        auto execute_program_movement = [this, &psteps, &sa, &steps_generator_drv, &variable_speed_driver](movement::path_intent_t program_) {
            /**
             * first, convert intentions into plan to execute (plan_to_execute)
             * plan_to_execute should be optimized for best movements (accelerations, breaks, corners optimization and so on)
             * then generate steps for each fragment and collect them to have one full movements fragment.
             * and last execute everything at once using stepping object.
             */
            auto plan_to_execute = variable_speed_driver.intent_to_movement_plan(program_);
            std::vector<hardware::multistep_command> ticks_to_execute;
            steps_generator_drv.movement_plan_to_step_commands(plan_to_execute, _gcdobjs.configuration.tick_duration(),
                [&, this](std::unique_ptr<std::vector<hardware::multistep_command>> msteps_p) {
                    ticks_to_execute.insert(ticks_to_execute.end(), msteps_p.get()->begin(), msteps_p.get()->end());
                    //std::cout << "steps to execute: " << msteps_p.get()->size() << std::endl;
                });
            
            _gcdobjs.stepping.get()->exec(ticks_to_execute);
            psteps = psteps + sa.steps_from_tick(ticks_to_execute,sa.get_last_tick_index(ticks_to_execute));
            std::cout << "position is " << psteps << std::endl;
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
                _gcdobjs.hardware_driver.get()->enable_steppers(std::get<movement::path_intentions::motor_t>(element).motor);
                break;
            default:
                throw std::invalid_argument("wrong path_intention");
            }
        }
        //_gcdobjs.hardware_driver.get()->enable_steppers({false});
    }
}; // namespace gcd


} // namespace gcd
} // namespace raspigcd

#endif
