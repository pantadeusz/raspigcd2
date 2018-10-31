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
#include <hardware_motor_layout.hpp>
#include <hardware_stepping_commands.hpp>
#include <list>
#include <memory>
#include <movement_simple_steps.hpp>
#include <steps_t.hpp>
#include <tuple>
#include <sstream>


namespace raspigcd {
namespace movement {

// using variable_speed_callback_i = std::function<void()>;
using var_speed_pointspeed_t = std::pair<distance_t, double>;

class var_speed_intentions_t
{
public:
    distance_t p0;
    distance_t p1;
    double intended_speed;
};


class variable_speed
{
protected:
    hardware::motor_layout* _motor_layout;
    std::shared_ptr<hardware::motor_layout> _motor_layout_ptr;

    double _max_speed_no_accel;
    double _acceleration;
    double _max_speed;
    double _tick_duration;

public:
    void set_motor_layout(const std::shared_ptr<hardware::motor_layout> ml) {}
    void set_max_speed_no_accel(const double& max_speed_no_accel_) { _max_speed_no_accel = max_speed_no_accel_; }
    void set_acceleration(const double& acceleration_) { _acceleration = acceleration_; }
    void set_max_speed(const double& max_speed_) { _max_speed = max_speed_; }
    void set_tick_duration(const double& tick_duration_) { _tick_duration = tick_duration_; }

    variable_speed(std::shared_ptr<hardware::motor_layout> ml, double max_speed_no_accel_, double acceleration_, double max_speed_, double tick_duration_) : _max_speed_no_accel(max_speed_no_accel_),
                                                                                                                                                             _acceleration(acceleration_),
                                                                                                                                                             _max_speed(max_speed_),
                                                                                                                                                             _tick_duration(tick_duration_)
    {
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
    //std::vector<hardware::multistep_command> goto_xyz(const var_speed_pointspeed_t p0, const var_speed_pointspeed_t p1, double dt){};

    // converts speed intetions into list of var_speed_pointspeed_t. That is taking into account machine limits
    std::list<var_speed_pointspeed_t> movement_intet_to_point_speeds(const std::list<var_speed_intentions_t>& intentions_)
    {
        std::list<var_speed_pointspeed_t> return_list_of_speedpoints;
        var_speed_intentions_t const *prev_intention = &( intentions_.front());
        std::cout << "Intentions size " << intentions_.size() << std::endl;
        for (const auto& intent : intentions_) {
            if (((prev_intention) != &intent) && (prev_intention->p1 != intent.p0)) {
                std::stringstream ss;
                ss << "second point of previous intetnion should equal first point of next intetnion: ";
                ss << prev_intention->p1 << " ..  " << intent.p0 ;
                throw std::invalid_argument(ss.str());
            }
            if (intent.intended_speed <= _max_speed_no_accel) {
                const var_speed_pointspeed_t a = {intent.p0, intent.intended_speed};
                const var_speed_pointspeed_t b = {intent.p1, intent.intended_speed};
                if (return_list_of_speedpoints.size() == 0) {
                    return_list_of_speedpoints.push_back(a);
                    return_list_of_speedpoints.push_back(b);
                } else {
                    if (a.second != return_list_of_speedpoints.back().second) return_list_of_speedpoints.push_back(a);
                    return_list_of_speedpoints.push_back(b);
                }
            }
            prev_intention = &intent;
        }
        return return_list_of_speedpoints;
    }
};

} // namespace movement
} // namespace raspigcd

#endif
