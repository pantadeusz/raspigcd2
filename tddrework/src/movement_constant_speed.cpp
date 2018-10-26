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

#include <cmath>
#include <configuration.hpp>
#include <distance_t.hpp>
#include <hardware_motor_layout.hpp>
#include <hardware_stepping_commands.hpp>
#include <memory>
#include <movement_constant_speed.hpp>
#include <movement_simple_steps.hpp>
#include <steps_t.hpp>


namespace raspigcd {
namespace movement {

void constant_speed::set_motor_layout(std::shared_ptr<hardware::motor_layout> ml)
{
    _motor_layout_ptr = ml;
    _motor_layout = _motor_layout_ptr.get();
}

constant_speed::constant_speed(std::shared_ptr<hardware::motor_layout> ml)
{
    set_motor_layout(ml);
}
/**
     * @brief 
     * 
     * @param p0 
     * @param p1 
     * @param v 
     * @param dt 
     * @return std::vector<hardware::multistep_command> 
     */
std::vector<hardware::multistep_command> constant_speed::goto_xyz(const distance_t p0, const distance_t p1, double v, double dt)
{
    std::vector<hardware::multistep_command> ret;
    distance_t vp = p1 - p0;
    double s = std::sqrt(vp.length2());
    distance_t vp_norm = (vp / s) * v;

    double T = s / v;
    double t = 0;

    ret.reserve(T / dt);

    steps_t p0steps = _motor_layout->cartesian_to_steps(p0);
    steps_t p_steps = p0steps;
    steps_t p1steps = _motor_layout->cartesian_to_steps(p1);
    for (int i = 0; t < T; ++i, t = dt * i) {
        auto pos = _motor_layout->cartesian_to_steps(p0 + vp_norm * t);
        std::vector<hardware::multistep_command> steps_to_add = simple_steps::chase_steps(p_steps, pos);
        ret.insert(ret.end(), steps_to_add.begin(), steps_to_add.end());
        p_steps = pos;
    }
    std::vector<hardware::multistep_command> steps_to_add = simple_steps::chase_steps(p_steps, p1steps);
    ret.insert(ret.end(), steps_to_add.begin(), steps_to_add.end());
    return ret;
}


} // namespace movement
} // namespace raspigcd
