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

#ifndef __RASPIGCD_MOVEMENT_PATH_INTENT_T_T_HPP__
#define __RASPIGCD_MOVEMENT_PATH_INTENT_T_T_HPP__

#include <cmath>
#include <configuration.hpp>
#include <distance_t.hpp>
#include <hardware/motor_layout.hpp>
#include <hardware/stepping_commands.hpp>
#include <memory>
#include <movement/simple_steps.hpp>
#include <steps_t.hpp>
#include <variant>
#include <functional>


namespace raspigcd {
namespace movement {

namespace path_intentions {
    using move_t = double; // movement is represented by intended speed
    struct pause_t {
        double delay_s; // delay is represented by intended delay in seconds
    };
    struct wait_key_t {
        double timeout_s; // wait for key (timeout in seconds)
        std::vector <int> buttons; // what buttons are to be checked (numbers the same as in low_buttons and setup in configuration)
    };
    struct spindle_t {
        double delay_s; // how long to wait for spindle to start/stop
        std::vector <double> spindle; // the motors speeds 0.0-1.0
    };
    struct motor_t {
        double delay_s; // how long to wait for motor to start/stop
        std::vector <bool> motor; // the motors state (on or off)
    };
}
/// path intent represents interpreted gcode into actual commands that are ready to execute
using path_intent_element_t = std::variant<distance_t, path_intentions::move_t, path_intentions::pause_t, path_intentions::spindle_t, path_intentions::motor_t>;
using path_intent_t = std::list<path_intent_element_t>;

} // namespace movement
} // namespace raspigcd

#endif
