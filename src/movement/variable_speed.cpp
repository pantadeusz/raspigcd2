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

#include <cassert>

namespace raspigcd {
namespace movement {


void variable_speed::set_motor_layout(const std::shared_ptr<hardware::motor_layout> ml)
{
    _motor_layout = ml.get();
    _motor_layout_ptr = ml;
}

double variable_speed::max_speed_no_accel(const distance_t& norm_vect) const
{
    return calculate_linear_coefficient_from_limits(_max_speed_no_accel, norm_vect);
}
double variable_speed::acceleration(const distance_t& norm_vect) const
{
    return calculate_linear_coefficient_from_limits(_acceleration, norm_vect);
}
double variable_speed::max_speed(const distance_t& norm_vect) const
{
    return calculate_linear_coefficient_from_limits(_max_speed, norm_vect);
}


void variable_speed::set_max_speed_no_accel(const std::vector<double>& max_speed_no_accel_) { _max_speed_no_accel = max_speed_no_accel_; }
void variable_speed::set_acceleration(const std::vector<double>& acceleration_) { _acceleration = acceleration_; }
void variable_speed::set_max_speed(const std::vector<double>& max_speed_) { _max_speed = max_speed_; }
void variable_speed::set_tick_duration(const double& tick_duration_) { _tick_duration = tick_duration_; }

variable_speed::variable_speed(
    std::shared_ptr<hardware::motor_layout> ml,
    std::vector<double> max_speed_no_accel_,
    std::vector<double> acceleration_,
    std::vector<double> max_speed_,
    double tick_duration_) : _max_speed_no_accel(max_speed_no_accel_),
                             _acceleration(acceleration_),
                             _max_speed(max_speed_),
                             _tick_duration(tick_duration_)
{
    _motor_layout = ml.get();
    _motor_layout_ptr = ml;
}
// given speed, target speed and acceleration, it calculate distance that it will be accelerating
double variable_speed::accleration_length_calc(double speed_0, double speed_1, double acceleration)
{
    double t_AM = (speed_1 - speed_0) / acceleration;
    double l_AM = std::abs(speed_0 * t_AM + acceleration * t_AM * t_AM / 2.0);
    return l_AM;
};


double variable_speed::calculate_linear_coefficient_from_limits(const std::vector<double>& limits_for_axes, const distance_t& norm_vect)
{
    assert(std::sqrt(norm_vect.length2()) > 0.9999);
    assert(std::sqrt(norm_vect.length2()) < 1.0001);
    double average_max_accel = 0;
    {
        double average_max_accel_sum = 0;
        for (unsigned int i = 0; i < limits_for_axes.size(); i++) {
            average_max_accel += limits_for_axes.at(i) * norm_vect.at(i);
            average_max_accel_sum += norm_vect.at(i);
        }
        average_max_accel = average_max_accel / average_max_accel_sum;
    }
    return average_max_accel;
};



// converts speed intetions into list of var_speed_pointspeed_t. That is taking into account machine limits
movement_plan_t variable_speed::intent_to_movement_plan(
    const std::list<std::variant<distance_t, double>>& intentions_)
{
    movement_plan_t ret;
    distance_t prev_pos;
    distance_t next_pos;
    double intended_velocity = 1000000;
    for (const auto& ie : intentions_) {
        if (ie.index() == 0) {
            next_pos = std::get<distance_t>(ie);
            if (ret.size() == 0) {
                ret.push_back(next_pos);
            } else if (!(next_pos == prev_pos)) { // ignore empty moves
                auto movement_vector_whole = next_pos - prev_pos;
                auto movement_vector_length = std::sqrt(movement_vector_whole.length2());
                auto movement_direction_vect = movement_vector_whole / movement_vector_length;

                if (intended_velocity <= max_speed_no_accel(movement_direction_vect)) {
                    ret.push_back(transition_t{.v0 = intended_velocity, .accel = 0, .max_v = intended_velocity});
                    ret.push_back(next_pos);
                } else {
                    double accel_length = accleration_length_calc(max_speed_no_accel(movement_direction_vect), intended_velocity, acceleration(movement_direction_vect));
                    if ((accel_length * 2.0) >= movement_vector_length) { // we can only accelerate and break
                        ret.push_back(transition_t{.v0 = max_speed_no_accel(movement_direction_vect), .accel = 0, .max_v = intended_velocity});
                        ret.push_back(next_pos);
                    } else {

                        distance_t inter_pos1 = movement_direction_vect * accel_length + prev_pos;
                        distance_t inter_pos2 = next_pos - (movement_direction_vect * accel_length);

                        ret.push_back(transition_t{.v0 = max_speed_no_accel(movement_direction_vect), .accel = acceleration(movement_direction_vect), .max_v = intended_velocity});
                        ret.push_back(inter_pos1);

                        ret.push_back(transition_t{.v0 = intended_velocity, .accel = 0, .max_v = intended_velocity});
                        ret.push_back(inter_pos2);

                        ret.push_back(transition_t{.v0 = intended_velocity, .accel = - acceleration(movement_direction_vect), .max_v = intended_velocity});
                        ret.push_back(next_pos);
                    }
                }
            }
            prev_pos = next_pos;
        } else if (ie.index() == 1) {
            intended_velocity = std::get<double>(ie);
        }
    }
    return ret;
}


} // namespace movement
} // namespace raspigcd
