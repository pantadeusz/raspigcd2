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
#include <vector>

#include <hardware_dof_conf.hpp>

namespace raspigcd {
namespace hardware {

struct single_step_command {
    bool step : 1, dir : 1;//, sync_laser_en: 1, sync_laser: 1;
};

struct multistep_command {
    single_step_command b[RASPIGCD_HARDWARE_DOF]; // duplicate of first b
    int count;                                    // number of times to repeat the command, it means that the command will be executed repeat n.
};
using multistep_commands_t = std::vector<multistep_command>;

inline bool operator==(const single_step_command &a, const single_step_command &b) {
    return (a.step == b.step) && (a.dir == b.dir);
    //return *(char*)&a == *(char*)&b;
}

} // namespace hardware
} // namespace raspigcd

#endif
