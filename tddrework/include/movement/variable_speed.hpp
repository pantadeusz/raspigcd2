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
#include <sstream>
#include <steps_t.hpp>
#include <tuple>
#include <variant>


namespace raspigcd {
namespace movement {

/**
 * @brief this is the node of path that supports variable speeds and accelerations
 * 
 */
struct var_speed_pointspeed_t {
    distance_t p; // the start position

    double v0;    // initial velocity
    double accel; // acceleration to the next node
    double max_v; // maximal velocity that can be performed on this fragment. This is from var_speed_intentions_t
};                // = std::pair<distance_t, double>;

struct transition_t {
    double v0;    // initial velocity
    double accel; // acceleration to the next node
    double max_v; // maximal intended velocity that can be performed on this fragment. This is from var_speed_intentions_t
};

/**
 * @brief This represents intentions for movement steps. The accelerations can be infinite here.
 */
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

    variable_speed(
        std::shared_ptr<hardware::motor_layout> ml,
        double max_speed_no_accel_,
        double acceleration_,
        double max_speed_,
        double tick_duration_) : _max_speed_no_accel(max_speed_no_accel_),
                                 _acceleration(acceleration_),
                                 _max_speed(max_speed_),
                                 _tick_duration(tick_duration_)
    {
    }
    // given speed, target speed and acceleration, it calculate distance that it will be accelerating
    double accleration_length_calc(double speed_0, double speed_1, double acceleration)
    {
        double t_AM = (speed_1 - speed_0) / acceleration;
        double l_AM = std::abs(speed_0 * t_AM + acceleration * t_AM * t_AM / 2.0);
        return l_AM;
    };

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
        var_speed_intentions_t const* prev_intention = &(intentions_.front());
        for (const auto& intent : intentions_) {
            if (((prev_intention) != &intent) && (prev_intention->p1 != intent.p0)) {
                std::stringstream ss;
                ss << "second point of previous intetnion should equal first point of next intetnion: ";
                ss << prev_intention->p1 << " ..  " << intent.p0;
                throw std::invalid_argument(ss.str());
            }

            var_speed_pointspeed_t a = {intent.p0, intent.intended_speed, 0, intent.intended_speed};
            var_speed_pointspeed_t b = {intent.p1, intent.intended_speed, 0, intent.intended_speed};
            if (return_list_of_speedpoints.size() == 0) {
                if (a.v0 > _max_speed_no_accel) {
                    a.v0 = _max_speed_no_accel;
                }
                return_list_of_speedpoints.push_back(a);
            }
            if (intent.intended_speed <= _max_speed_no_accel) {
                if (a.v0 != return_list_of_speedpoints.back().v0)
                    return_list_of_speedpoints.push_back(a);
                else
                    return_list_of_speedpoints.back().max_v = intent.intended_speed;
                return_list_of_speedpoints.push_back(b);
            } else { // perform acceleration and break - first approximation. Next we can try to optimize it
                auto movement_vector_whole = (b.p - a.p);
                auto movement_vector_length = std::sqrt(movement_vector_whole.length2());
                auto movement_direction_vect = movement_vector_whole / movement_vector_length;

                double intended_speed = intent.intended_speed;
                double accel_length = accleration_length_calc(_max_speed_no_accel, intended_speed, _acceleration);
                if ((accel_length * 2.0) >= movement_vector_length) { // we can only accelerate and break
                    // TODO: auto top_speed =
                    // TODO: return_list_of_speedpoints.push_back(b);

                } else { // we need to accelerate, move with constant speed, and then break
                    //double T = (intended_speed - _max_speed_no_accel)/_acceleration; // time of acceleration and break
                    auto mvect = movement_direction_vect * accel_length; // (_max_speed_no_accel*T+_acceleration*T*T/2.0);
                    var_speed_pointspeed_t a1 = {intent.p0 + mvect, intended_speed, 0, intent.intended_speed};
                    var_speed_pointspeed_t b1 = {intent.p1 - mvect, intended_speed, -_acceleration, intent.intended_speed};
                    return_list_of_speedpoints.back().accel = _acceleration;
                    return_list_of_speedpoints.back().max_v = intent.intended_speed;
                    return_list_of_speedpoints.push_back(a1);
                    return_list_of_speedpoints.push_back(b1);
                    b.v0 = _max_speed_no_accel;
                    b.accel = 0;
                    return_list_of_speedpoints.push_back(b);
                }
            }
            prev_intention = &intent;
        }
        return return_list_of_speedpoints;
    }

    // converts speed intetions into list of var_speed_pointspeed_t. That is taking into account machine limits
    std::list<std::variant<distance_t,transition_t> > movement_intet_to_point_speeds_v(
            const std::list<std::variant<distance_t,double> >& intentions_)
    {
        std::list<std::variant<distance_t,transition_t> > ret;
        bool is_coord = true;
        distance_t prev_pos;
        distance_t next_pos;
        double intended_velocity = 1000000;
        for (const auto &ie:intentions_) {
            if (is_coord) {
                next_pos = std::get<distance_t>(ie);
                if (ret.size() == 0) {
                    ret.push_back(next_pos);
                } else if (!(next_pos == prev_pos)) { // ignore empty moves
                    if (intended_velocity <= _max_speed_no_accel) {
                        ret.push_back(transition_t{.v0=intended_velocity,.accel=0,.max_v=intended_velocity});
                        ret.push_back(next_pos);
                    } else {
                        auto movement_vector_whole = next_pos - prev_pos;
                        auto movement_vector_length = std::sqrt(movement_vector_whole.length2());
                        double accel_length = accleration_length_calc(_max_speed_no_accel, intended_velocity, _acceleration);
                        if ((accel_length * 2.0) >= movement_vector_length) { // we can only accelerate and break
                            ret.push_back(transition_t{.v0=_max_speed_no_accel,.accel=0,.max_v=intended_velocity});
                            ret.push_back(next_pos);
                        } else {
                            auto movement_direction_vect = movement_vector_whole / movement_vector_length;

                            distance_t inter_pos1 = movement_direction_vect*accel_length + prev_pos;
                            distance_t inter_pos2 = next_pos - (movement_direction_vect*accel_length);

                            ret.push_back(transition_t{.v0=_max_speed_no_accel,.accel=_acceleration,.max_v=intended_velocity});
                            ret.push_back(inter_pos1);

                            ret.push_back(transition_t{.v0=intended_velocity,.accel=0,.max_v=intended_velocity});
                            ret.push_back(inter_pos2);

                            ret.push_back(transition_t{.v0=intended_velocity,.accel=-_acceleration,.max_v=intended_velocity});
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
};

//std::list<std::variant<distance_t,transition_t> >
auto get_path_length = [](const auto &path) -> double {
    double l = 0;
    if (path.size() > 0) {
        auto prev = path.front();
        for (const auto &e : path) {
            try {
                l += std::sqrt((std::get<distance_t>(e) - std::get<distance_t>(prev)).length2()); prev = e;
            } catch (...) {}
        }
    }
    return l;
};


} // namespace movement
} // namespace raspigcd

#endif
