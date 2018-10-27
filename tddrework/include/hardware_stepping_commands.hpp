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

#ifndef __RASPIGCD_HARDWARE_STEPPING_COMMANDS_T_HPP__
#define __RASPIGCD_HARDWARE_STEPPING_COMMANDS_T_HPP__

#include <cstdint>

#ifndef RASPIGCD_HARDWARE_DOF
#define RASPIGCD_HARDWARE_DOF 4
#endif

namespace raspigcd {
namespace hardware {

struct single_step_command {
    unsigned char step : 1, dir : 1;
};

union multistep_command {
    struct {
        single_step_command b[RASPIGCD_HARDWARE_DOF]; // duplicate of first b
        int count;                                   // number of times to repeat the command, it means that the command will be executed repeat n.
    } cmnd;
    int64_t v;
};

} // namespace hardware
} // namespace raspigcd

#endif
