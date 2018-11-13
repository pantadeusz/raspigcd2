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

struct transition_t {
    double v0;    // initial velocity
    double accel; // acceleration to the next node
    double max_v; // maximal intended velocity that can be performed on this fragment. The most desired speed. The velocity cannot exceed max_v no matter what.
};

using delay_t = double; // delay in seconds
using movement_plan_element_t = std::variant<distance_t, transition_t, delay_t>;
using movement_plan_t = std::list<movement_plan_element_t>;




class steps_generator
{
protected:
    hardware::motor_layout* _motor_layout;
    std::shared_ptr<hardware::motor_layout> _motor_layout_ptr;

    std::vector<hardware::multistep_command> collapse_repeated_steps(const std::list<hardware::multistep_command>& commands) const;

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
    [[deprecated("This function is actualy not used anywhere and calls movement_from_to(p0, transition, p1, dt)")]]
    std::vector<hardware::multistep_command> goto_xyz(const distance_t p0, const distance_t p1, double v, double dt) const;

    /**
     * @brief calculates steps series to move from point p0 to point p1 with given transition
     * 
     * This method throws exception if velocity could exceed max_v
     * 
     * @param p0 initial position
     * @param p1 destination position
     * @param transition the transition between positions (velocity, acceleration and max velocity) 
     * @param dt time step in seconds
     * @return std::vector<hardware::multistep_command> series of multistep_command values for stepper motors
     */
    std::vector<hardware::multistep_command> movement_from_to(const distance_t& p0, const transition_t& transition, const distance_t& p1, const double dt) const;


    /**
     * @brief Generates multistep commands to execute whole execution plan.
     * 
     * @param consumer_f_ the callback function that takes generated vector of multistep_command s for each segment of p0-(transition/delay)-p1
     * 
     */
    void movement_plan_to_step_commands(const movement_plan_t &plan_to_execute, const double dt, 
        std::function < void (std::unique_ptr<std::vector<hardware::multistep_command> > ) > consumer_f_) const ;


    /**
     * @brief calculates the end velocity for given movement
     * 
     */
    double velocity_after_from_to(const distance_t& p0, const transition_t& transition, const distance_t& p1, const double dt) const;
};

} // namespace movement
} // namespace raspigcd

#endif
