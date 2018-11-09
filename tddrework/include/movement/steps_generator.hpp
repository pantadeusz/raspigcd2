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

#ifndef __RASPIGCD_MOVEMENT_ACCELERATIONS_T_HPP__
#define __RASPIGCD_MOVEMENT_ACCELERATIONS_T_HPP__

#include <configuration.hpp>
#include <distance_t.hpp>
#include <steps_t.hpp>
#include <hardware/stepping_commands.hpp>
#include <movement/simple_steps.hpp>
#include <hardware/motor_layout.hpp>
#include <memory>
#include <cmath>
#include <variant>


namespace raspigcd {
namespace movement {

struct transition_t {
    double v0;    // initial velocity
    double accel; // acceleration to the next node
    double max_v; // maximal intended velocity that can be performed on this fragment. The most desired speed. The velocity cannot exceed max_v no matter what.
};

using movement_plan_t = std::list<std::variant<distance_t,transition_t> >;


class steps_generator {
    protected:
    hardware::motor_layout *_motor_layout;
    std::shared_ptr<hardware::motor_layout> _motor_layout_ptr;

    std::vector<hardware::multistep_command> collapse_repeated_steps(const std::list < hardware::multistep_command > &commands) const;

    public:
    void set_motor_layout(std::shared_ptr<hardware::motor_layout> ml);

    steps_generator(std::shared_ptr<hardware::motor_layout> ml);
    /**
     * @brief 
     * 
     * @param p0 
     * @param p1 
     * @param v 
     * @param dt 
     * @return std::vector<hardware::multistep_command> 
     */
    std::vector<hardware::multistep_command> goto_xyz(const distance_t p0, const distance_t p1, double v, double dt) const ;

    std::vector<hardware::multistep_command> movement_from_to(const distance_t &p0,const transition_t &transition,const distance_t &p1, const double dt) const;
};

} // namespace hardware
} // namespace raspigcd

#endif
