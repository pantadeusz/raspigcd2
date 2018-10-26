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

#ifndef __RASPIGCD_HARDWARE_STEPPING_T_HPP__
#define __RASPIGCD_HARDWARE_STEPPING_T_HPP__

#include <configuration.hpp>
#include <distance_t.hpp>
#include <hardware_stepping_commands.hpp>
#include <hardware_low_steppers.hpp>
#include <steps_t.hpp>
#include <functional>
#include <memory>

namespace raspigcd {
namespace hardware {

class stepping {
public:
	/**
	* @brief Executes multistep commands list. On each step it calls on_step_ function with current position as an argument. The return value is the final steps value
	*
	* @param start_steps the initial position of motors in steps
	* @param commands_to_do array of commands to execute
	* @param on_step_ function to execute on each step. This function can throw exceptions to break execution of steps
	* @return steps_t final position of the machine in steps
	*/
	virtual steps_t exec( const steps_t& start_steps, const std::vector<multistep_command>& commands_to_do, std::function<void( const steps_t& )> on_step_ = []( const steps_t& ) {} ) = 0;
};

class stepping_sim : public stepping {
public:
	steps_t exec( const steps_t& start_steps, const std::vector<multistep_command>& commands_to_do, std::function<void( const steps_t& )> on_step_ );
};


class stepping_simple_timer : public stepping {
public:
    int _delay_microseconds;
    std::shared_ptr<low_steppers> _steppers_driver_shr;
    low_steppers *_steppers_driver;

    /**
     * @brief Set the delay in microseconds
     * 
     * @param delay_ms the intended minimal step delay
     */
    void set_delay_microseconds(int delay_ms);

    /**
     * @brief Set the low level steppers driver
     * 
     * @param steppers_driver pointer to the steppers driver. This can be faked or true
     */
    void set_low_level_steppers_driver(std::shared_ptr<low_steppers> steppers_driver);

	steps_t exec( const steps_t& start_steps, const std::vector<multistep_command>& commands_to_do, std::function<void( const steps_t& )> on_step_ );

    stepping_simple_timer(int delay_ms, std::shared_ptr<low_steppers> steppers_driver) {
        set_delay_microseconds(delay_ms);
        set_low_level_steppers_driver(steppers_driver);
    }
};

} // namespace hardware
} // namespace raspigcd

#endif
