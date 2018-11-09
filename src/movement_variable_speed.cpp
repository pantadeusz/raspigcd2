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


namespace raspigcd {
namespace movement {


void variable_speed::set_motor_layout(const std::shared_ptr<hardware::motor_layout> ml)
{
    _motor_layout = ml.get();
    _motor_layout_ptr = ml;
}
void variable_speed::set_max_speed_no_accel(const double& max_speed_no_accel_) { _max_speed_no_accel = max_speed_no_accel_; }
void variable_speed::set_acceleration(const double& acceleration_) { _acceleration = acceleration_; }
void variable_speed::set_max_speed(const double& max_speed_) { _max_speed = max_speed_; }
void variable_speed::set_tick_duration(const double& tick_duration_) { _tick_duration = tick_duration_; }

variable_speed::variable_speed(
    std::shared_ptr<hardware::motor_layout> ml,
    double max_speed_no_accel_,
    double acceleration_,
    double max_speed_,
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

// converts speed intetions into list of var_speed_pointspeed_t. That is taking into account machine limits
movement_plan_t variable_speed::intent_to_movement_plan(
    const std::list<std::variant<distance_t, double>>& intentions_)
{
    movement_plan_t ret;
    bool is_coord = true;
    distance_t prev_pos;
    distance_t next_pos;
    double intended_velocity = 1000000;
    for (const auto& ie : intentions_) {
        if (is_coord) {
            next_pos = std::get<distance_t>(ie);
            if (ret.size() == 0) {
                ret.push_back(next_pos);
            } else if (!(next_pos == prev_pos)) { // ignore empty moves
                if (intended_velocity <= _max_speed_no_accel) {
                    ret.push_back(transition_t{.v0 = intended_velocity, .accel = 0, .max_v = intended_velocity});
                    ret.push_back(next_pos);
                } else {
                    auto movement_vector_whole = next_pos - prev_pos;
                    auto movement_vector_length = std::sqrt(movement_vector_whole.length2());
                    double accel_length = accleration_length_calc(_max_speed_no_accel, intended_velocity, _acceleration);
                    if ((accel_length * 2.0) >= movement_vector_length) { // we can only accelerate and break
                        ret.push_back(transition_t{.v0 = _max_speed_no_accel, .accel = 0, .max_v = intended_velocity});
                        ret.push_back(next_pos);
                    } else {
                        auto movement_direction_vect = movement_vector_whole / movement_vector_length;

                        distance_t inter_pos1 = movement_direction_vect * accel_length + prev_pos;
                        distance_t inter_pos2 = next_pos - (movement_direction_vect * accel_length);

                        ret.push_back(transition_t{.v0 = _max_speed_no_accel, .accel = _acceleration, .max_v = intended_velocity});
                        ret.push_back(inter_pos1);

                        ret.push_back(transition_t{.v0 = intended_velocity, .accel = 0, .max_v = intended_velocity});
                        ret.push_back(inter_pos2);

                        ret.push_back(transition_t{.v0 = intended_velocity, .accel = -_acceleration, .max_v = intended_velocity});
                        ret.push_back(next_pos);
                    }
                }
            }
            prev_pos = next_pos;
        } else {
            intended_velocity = std::get<double>(ie);
        }
        is_coord = !is_coord;
    }
    return ret;
}


} // namespace movement
} // namespace raspigcd
