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

#ifndef __RASPIGCD_MOVEMENT_VARIABLE_SPEED_T_HPP__
#define __RASPIGCD_MOVEMENT_VARIABLE_SPEED_T_HPP__

#include <cmath>
#include <configuration.hpp>
#include <distance_t.hpp>
#include <hardware/motor_layout.hpp>
#include <hardware/stepping_commands.hpp>
#include <list>
#include <memory>
#include <movement/simple_steps.hpp>
#include <movement/steps_generator.hpp>
#include <sstream>
#include <steps_t.hpp>
#include <tuple>


namespace raspigcd {
namespace movement {




class variable_speed
{
protected:
    hardware::motor_layout* _motor_layout;
    std::shared_ptr<hardware::motor_layout> _motor_layout_ptr;

    std::vector<double> _max_speed_no_accel;
    std::vector<double> _acceleration;
    std::vector<double> _max_speed;
    double _tick_duration;


    double max_speed_no_accel(const distance_t& norm_vect) const;
    double acceleration(const distance_t& norm_vect) const;
    double max_speed(const distance_t& norm_vect) const;

public:
    void set_motor_layout(const std::shared_ptr<hardware::motor_layout> ml);
    void set_max_speed_no_accel(const std::vector<double>& max_speed_no_accel_);
    void set_acceleration(const std::vector<double>& acceleration_);
    void set_max_speed(const std::vector<double>& max_speed_);
    void set_tick_duration(const double& tick_duration_);

    variable_speed(
        std::shared_ptr<hardware::motor_layout> ml,
        std::vector<double> max_speed_no_accel_,
        std::vector<double> acceleration_,
        std::vector<double> max_speed_,
        double tick_duration_);
    // given speed, target speed and acceleration, it calculate distance that it will be accelerating
    double accleration_length_calc(double speed_0, double speed_1, double acceleration);

    // converts speed intetions into list of var_speed_pointspeed_t. That is taking into account machine limits
    movement_plan_t intent_to_movement_plan(
        const path_intent_t& intentions_);

    // calulates maximal linear value given the maximal values for each axis, and the normal vector of intended move
    // it works that if norm_vect is 1 on one axis, then the value from limits_for_axes on this
    // otherwise it blends, so if it is a limit, then applying linear limit will not exceed limits
    // it is used to calculate maximal speed for movement in any direction, as well as maximal acceleration and speed without acceleration
    static double calculate_linear_coefficient_from_limits(const std::vector<double>& limits_for_axes, const distance_t& norm_vect);
};

//movement_plan_t
auto get_path_length = [](const auto& path) -> double {
    double l = 0;
    if (path.size() > 0) {
        auto prev = path.front();
        for (const auto& e : path) {
            try {
                l += std::sqrt((std::get<distance_t>(e) - std::get<distance_t>(prev)).length2());
                prev = e;
            } catch (...) {
            }
        }
    }
    return l;
};


} // namespace movement
} // namespace raspigcd

#endif
