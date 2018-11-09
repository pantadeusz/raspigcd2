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
#include <movement/steps_generator.hpp>
#include <movement/simple_steps.hpp>
#include <hardware/motor_layout.hpp>
#include <hardware/stepping_commands.hpp>
#include <memory>
#include <steps_t.hpp>


namespace raspigcd {
namespace movement {

void steps_generator::set_motor_layout(std::shared_ptr<hardware::motor_layout> ml)
{
    _motor_layout_ptr = ml;
    _motor_layout = _motor_layout_ptr.get();
}

steps_generator::steps_generator(std::shared_ptr<hardware::motor_layout> ml)
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
std::vector<hardware::multistep_command> steps_generator::goto_xyz(const distance_t p0, const distance_t p1, double v, double dt) const 
{
    std::list<hardware::multistep_command> ret;
    distance_t vp = p1 - p0;
    double s = std::sqrt(vp.length2());
    distance_t vp_norm = (vp / s) * v;

    double T = s / v;
    double t = 0;


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
    if (ret.size() == 0) return {};
    std::vector<hardware::multistep_command> ret_vect;
    ret_vect.reserve(ret.size());
    // repeated commands should be one with apropriate count
    for (auto &e: ret) {
        if ((ret_vect.size() == 0) || (e.v != ret_vect.back().v)) {
            ret_vect.push_back(e);
        } else {
            if (ret_vect.back().cmnd.count > 0x0fffffff) {
                ret_vect.push_back(e);
            } else {
                ret_vect.back().cmnd.count++;
            }
        }
    }
    return ret_vect;
}


std::vector<hardware::multistep_command> steps_generator::goto_complete(const distance_t &p0,const transition_t &transition,const distance_t &p1, const double dt) const {
    std::list<hardware::multistep_command> ret;
    distance_t vp = p1 - p0;
    double s = std::sqrt(vp.length2());
    distance_t vp_norm = (vp / s) * transition.v0;

    double T = s / transition.v0;
    double t = 0;

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
    if (ret.size() == 0) return {};
    std::vector<hardware::multistep_command> ret_vect;
    ret_vect.reserve(ret.size());
    // repeated commands should be one with apropriate count
    for (auto &e: ret) {
        if ((ret_vect.size() == 0) || (e.v != ret_vect.back().v)) {
            ret_vect.push_back(e);
        } else {
            if (ret_vect.back().cmnd.count > 0x0fffffff) {
                ret_vect.push_back(e);
            } else {
                ret_vect.back().cmnd.count++;
            }
        }
    }
    return ret_vect;
}



} // namespace movement
} // namespace raspigcd
