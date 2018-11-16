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

#include <gcd/factory.hpp>
#include <hardware/motor_layout.hpp>
#include <hardware/driver/raspberry_pi.hpp>
#include <hardware/driver/inmem.hpp>
#include <hardware/stepping.hpp>
#include <hardware/stepping_commands.hpp>
#include <movement/simple_steps.hpp>
#include <movement/variable_speed.hpp>
#include <movement/path_intent_t.hpp>
#include <memory>

namespace raspigcd {
namespace gcd {


gcode_interpreter_objects_t gcode_interpreter_objects_factory(const configuration::global &cfg, const machine_driver_selection driver_select_) {
    gcode_interpreter_objects_t gio_ret;
//std::shared_ptr<hardware::motor_layout> motor_layout;
    gio_ret.configuration = cfg;
    if (driver_select_ == RASPBERRY_PI) {
        try {
            gio_ret.buttons = std::make_shared< hardware::driver::raspberry_pi_3>(gio_ret.configuration);
            gio_ret.steppers = std::make_shared< hardware::driver::raspberry_pi_3>(gio_ret.configuration);
            gio_ret.spindles_pwm = std::make_shared< hardware::driver::raspberry_pi_3>(gio_ret.configuration);
            gio_ret.timers = std::make_shared< hardware::driver::raspberry_pi_3>(gio_ret.configuration);
        } catch ( ... ) {
            std::cout << "falling back to emulation of hardware motors..." << std::endl;
            gio_ret.steppers = std::make_shared< hardware::driver::inmem >();
        }
    } else {
            gio_ret.steppers = std::make_shared< hardware::driver::inmem >();
    }
    gio_ret.motor_layout = hardware::motor_layout::get_instance(gio_ret.configuration);
    gio_ret.stepping = std::make_shared<hardware::stepping_simple_timer>(gio_ret.configuration, gio_ret.steppers);

    movement::steps_generator steps_generator_drv(gio_ret.motor_layout);
	movement::variable_speed variable_speed_driver( gio_ret.motor_layout, gio_ret.configuration, gio_ret.configuration.tick_duration() );

   // gio_ret.hardware_driver.get()->enable_steppers({true});

    movement::path_intent_t simple_program = {
			distance_t{0, 0, 0, 0}, 30.0,
			distance_t{0, 0, 5, 0}, 5.0,

			distance_t{0, 10, 5, 0}, 5.0,
			distance_t{10, 10, 5, 0}, 30.0,
			distance_t{10, 0, 5, 0}, 5.0,

			distance_t{0, 0, 0, 0}
		};
	auto plan_to_execute = variable_speed_driver.intent_to_movement_plan( simple_program );

    steps_t psteps = {0, 0, 0, 0};
    std::vector<hardware::multistep_command> ticks_to_execute;
    steps_generator_drv.movement_plan_to_step_commands(plan_to_execute, gio_ret.configuration.tick_duration(), 
        [&psteps,&gio_ret,&ticks_to_execute](std::unique_ptr<std::vector<hardware::multistep_command> > msteps_p){
            //ticks_to_execute.push_back(std::move(msteps_p));
            ticks_to_execute.insert(ticks_to_execute.end(), msteps_p.get()->begin(), msteps_p.get()->end());
            std::cout << "steps to execute: " << msteps_p.get()->size() << std::endl;

        });
    gio_ret.stepping.get()->exec( ticks_to_execute );      
  //  gio_ret.hardware_driver.get()->enable_steppers({false});
    return gio_ret;
}


} // namespace gcd
} // namespace raspigcd
