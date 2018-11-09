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


std::vector<hardware::multistep_command> steps_generator::collapse_repeated_steps(const std::list < hardware::multistep_command > &ret) const {
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
    ret_vect.shrink_to_fit();
    return ret_vect;
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
    transition_t transition = {
        .v0 = v,
        .accel = 0,
        .max_v = v
    };
    return movement_from_to(p0, transition, p1, dt);
}


std::vector<hardware::multistep_command> steps_generator::movement_from_to(const distance_t &p0,const transition_t &transition,const distance_t &p1, const double dt) const {
    std::list<hardware::multistep_command> ret;
    double v0 = transition.v0;
    distance_t vp = p1 - p0;
    double s = std::sqrt(vp.length2()); ///< summary distance to go
    distance_t vp_v = vp / s;

    //double T = s / v0;
    double a = transition.accel;
    double t = dt;  ///< current time
    auto l = [&](){return v0 * t + 0.5*a*t*t;}; ///< current distance from p0
    steps_t p0steps = _motor_layout->cartesian_to_steps(p0);
    steps_t p_steps = p0steps;
    steps_t p1steps = _motor_layout->cartesian_to_steps(p1);
    for (int i = 1; l() < s; ++i, t = dt * i) {
        auto pos = _motor_layout->cartesian_to_steps(p0 + vp_v * l());
        std::vector<hardware::multistep_command> steps_to_add = simple_steps::chase_steps(p_steps, pos);
        ret.insert(ret.end(), steps_to_add.begin(), steps_to_add.end());
        p_steps = pos;
        if ((a*t + v0) > transition.max_v) throw std::invalid_argument("velocity exceeds max_v");
    }
    std::vector<hardware::multistep_command> steps_to_add = simple_steps::chase_steps(p_steps, p1steps);
    ret.insert(ret.end(), steps_to_add.begin(), steps_to_add.end());

    return collapse_repeated_steps(ret);
}

double steps_generator::velocity_after_from_to(const distance_t &p0,const transition_t &transition,const distance_t &p1, const double dt) const {
    double v0 = transition.v0;
    distance_t vp = p1 - p0;
    double s = std::sqrt(vp.length2()); ///< summary distance to go
    //double T = s / v0;
    double a = transition.accel;
    double t = dt;  ///< current time
    auto l = [&](){return v0 * t + 0.5*a*t*t;}; ///< current distance from p0
    double ret_v = v0;
    for (int i = 1; l() < s; ++i, t = dt * i) ret_v = (a*t + v0);
    return ret_v;
}



} // namespace movement
} // namespace raspigcd
