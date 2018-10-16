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


#ifndef __RASPIGCD_STEP_SEQUENCE_EXECUTOR_T_HPP__
#define __RASPIGCD_STEP_SEQUENCE_EXECUTOR_T_HPP__

#include <configuration_t.hpp>
#include <array>
#include <steps_t.hpp>

/*

 This is the lowest level executor. It generates signals to pins


 */
namespace raspigcd
{

struct executor_motor_command_t
{
    unsigned char step : 1, dir : 1;
};

union executor_command_t {
    struct {
        executor_motor_command_t b[DEGREES_OF_FREEDOM]; // duplicate of first b
        int repeat; // number of times to repeat the command, it means that the command will be executed (repeat+1) times.
    } cmnd;
    executor_motor_command_t b[DEGREES_OF_FREEDOM];
    int64_t v;
};

class step_sequence_executor_t
{
  protected:

  public:
    /**
     * @brief set the value of steps from origin (in steps, not in mm or other unit)
     * 
     */
    virtual void set_position(const steps_t &steps) = 0;
    virtual steps_t get_position() const = 0;

    /**
   * executes list of commands.
   * 
   * if the program is already executing, then break
   * */

    virtual int execute(const std::vector<executor_command_t> &commands) = 0;
    /**
     * @brief turns on or off motors
     * 
     * @param en true - enable stepper motors, false - disable stepper motors
     */
    virtual void enable(bool en) = 0;

    virtual void terminate() = 0;

    /**
     * @brief generates steps based on given commands vector
     */
    static steps_t commands_to_steps(const std::vector<executor_command_t> &commands);

    // get instance
    static step_sequence_executor_t &get(configuration_t &cfg_);
};

} // namespace raspigcd

#endif