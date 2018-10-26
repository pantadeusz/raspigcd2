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

#ifndef __RASPIGCD_MOVEMENT_CONSTANT_SPEED_T_HPP__
#define __RASPIGCD_MOVEMENT_CONSTANT_SPEED_T_HPP__

#include <configuration.hpp>
#include <distance_t.hpp>
#include <steps_t.hpp>
#include <hardware_stepping_commands.hpp>
#include <movement_simple_steps.hpp>
#include <hardware_motor_layout.hpp>
#include <memory>
#include <cmath>


namespace raspigcd {
namespace movement {

class constant_speed {
    protected:
    hardware::motor_layout *_motor_layout;
    std::shared_ptr<hardware::motor_layout> _motor_layout_ptr;
    public:
    void set_motor_layout(std::shared_ptr<hardware::motor_layout> ml);

    constant_speed(std::shared_ptr<hardware::motor_layout> ml);
    /**
     * @brief 
     * 
     * @param p0 
     * @param p1 
     * @param v 
     * @param dt 
     * @return std::vector<hardware::multistep_command> 
     */
    std::vector<hardware::multistep_command> goto_xyz(const distance_t p0, const distance_t p1, double v, double dt);
};

} // namespace hardware
} // namespace raspigcd

#endif
