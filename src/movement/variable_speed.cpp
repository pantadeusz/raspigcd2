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
#include <hardware/motor_layout.hpp>
#include <hardware/stepping_commands.hpp>
#include <memory>
#include <movement/simple_steps.hpp>
#include <movement/variable_speed.hpp>
#include <steps_t.hpp>

#include <cmath>
#include <cassert>


namespace raspigcd {
namespace movement {


void variable_speed::set_motor_layout(const std::shared_ptr<hardware::motor_layout> ml)
{
    _motor_layout = ml.get();
    _motor_layout_ptr = ml;
}

void variable_speed::set_tick_duration(const double& tick_duration_) {
    _tick_duration = tick_duration_;
}



void variable_speed::set_limits(const configuration::limits &limits_) { 
    for (int i = 0; i < RASPIGCD_HARDWARE_DOF; i++) {
        if (std::isnan(limits_.max_velocity_mm_s[i])) throw std::invalid_argument("limits_.max_velocity_mm_s cannot contain nans");
        if (std::isnan(limits_.max_no_accel_velocity_mm_s[i])) throw std::invalid_argument("limits_.max_no_accel_velocity_mm_s cannot contain nans");
        if (std::isnan(limits_.max_accelerations_mm_s2[i])) throw std::invalid_argument("limits_.max_accelerations_mm_s2 cannot contain nans");
        if (limits_.max_velocity_mm_s[i] < 0.01) throw std::invalid_argument("limits_.max_velocity_mm_s cannot be less than 0.01");
        if (limits_.max_no_accel_velocity_mm_s[i] < 0.01) throw std::invalid_argument("limits_.max_no_accel_velocity_mm_s cannot be less than 0.01");
        if (limits_.max_accelerations_mm_s2[i] < 0.01) throw std::invalid_argument("limits_.max_accelerations_mm_s2 cannot be less than 0.01");
    }
    _limits = limits_;
}

variable_speed::variable_speed(
    std::shared_ptr<hardware::motor_layout> ml,
    const configuration::limits &limits_,
    double tick_duration_) : _tick_duration(tick_duration_)
{
    _motor_layout = ml.get();
    _motor_layout_ptr = ml;
    set_limits(limits_);
}
// given speed, target speed and acceleration, it calculate distance that it will be accelerating
double variable_speed::accleration_length_calc(double speed_0, double speed_1, double acceleration)
{
    double t_AM = (speed_1 - speed_0) / acceleration;
    double l_AM = std::abs(speed_0 * t_AM + acceleration * t_AM * t_AM / 2.0);
    return l_AM;
};





// converts speed intetions into list of var_speed_pointspeed_t. That is taking into account machine limits
movement_plan_t variable_speed::intent_to_movement_plan( const path_intent_t& intentions_)
{
    movement_plan_t ret;
    distance_t prev_pos;
    distance_t next_pos;
    double intended_velocity = 1000000;
    path_intent_element_t prev_ie;
    for (const auto& ie : intentions_) {
        if (ie.index() == 0) {
            next_pos = std::get<distance_t>(ie);
            if (ret.size() == 0) {
                ret.push_back(next_pos);
                prev_ie = ie;
            } else {
                if (prev_ie.index() == 1) {
                    if (!(next_pos == prev_pos)) { // ignore empty moves
                        auto movement_vector_whole = next_pos - prev_pos;
                        auto movement_vector_length = std::sqrt(movement_vector_whole.length2());
                        auto movement_direction_vect = movement_vector_whole / movement_vector_length;
    // 
                        if (intended_velocity <= _limits.proportional_max_no_accel_velocity_mm_s(movement_direction_vect)) {
                            ret.push_back(transition_t{.v0 = intended_velocity, .accel = 0, .max_v = intended_velocity});
                            ret.push_back(next_pos);
                        } else {
                            double accel_length = accleration_length_calc(_limits.proportional_max_no_accel_velocity_mm_s(movement_direction_vect), intended_velocity, _limits.proportional_max_accelerations_mm_s2( movement_direction_vect));
                            if ((accel_length * 2.0) >= movement_vector_length) { // we can only accelerate and break
                                ret.push_back(transition_t{.v0 = _limits.proportional_max_no_accel_velocity_mm_s(movement_direction_vect), .accel = 0, .max_v = intended_velocity});
                                ret.push_back(next_pos);
                            } else {

                                distance_t inter_pos1 = movement_direction_vect * accel_length + prev_pos;
                                distance_t inter_pos2 = next_pos - (movement_direction_vect * accel_length);

                                ret.push_back(transition_t{.v0 = _limits.proportional_max_no_accel_velocity_mm_s(movement_direction_vect), .accel = _limits.proportional_max_accelerations_mm_s2( movement_direction_vect), .max_v = intended_velocity});
                                ret.push_back(inter_pos1);

                                ret.push_back(transition_t{.v0 = intended_velocity, .accel = 0, .max_v = intended_velocity});
                                ret.push_back(inter_pos2);

                                ret.push_back(transition_t{.v0 = intended_velocity, .accel = - _limits.proportional_max_accelerations_mm_s2( movement_direction_vect), .max_v = intended_velocity});
                                ret.push_back(next_pos);
                            }
                        }
                    }
                }

                if (prev_ie.index() >= 2) {
                    // TODO
                    throw "variable_speed does not to delays!!";
                }

            }
            prev_pos = next_pos;
        } else if (ie.index() == 1) {
            intended_velocity = std::get<path_intentions::move_t>(ie);
        }
        prev_ie = ie;
    }
    return ret;
}


} // namespace movement
} // namespace raspigcd
