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

#ifndef __RASPIGCD_MOVEMENT_PHYSICS_HPP__
#define __RASPIGCD_MOVEMENT_PHYSICS_HPP__

#include <configuration.hpp>
#include <distance_t.hpp>
#include <hardware/stepping_commands.hpp>
#include <list>
#include <steps_t.hpp>

namespace raspigcd {
namespace movement {

/**
 * @brief the physics library that can be used to calculate accelerations, distances and other things.
 *
 * basic units are: mm, s
 */

namespace physics {


struct path_node_t {
    distance_t p; // position
    double v;     // velocity
};

/*
v1 = v0 + a*t
s1 = s0 + v0*t + a*t*t/2
*/

/**
 * @brief calculate velocity for given distance and acceleration.
 */
distance_t get_next_position(const distance_t &s0, const distance_t &v0, const double &a, const double &t);

/**
 * @brief returns next velocity
 * 
 */
distance_t get_next_velocity(const distance_t &s0, const distance_t &v0, const double &a, const double &t);

/**
 * @brief returns next velocity
 * 
 */
path_node_t get_next_node(const distance_t &s0, const distance_t &v0, const double &a, const double &t);

std::pair<distance_t,distance_t> get_next_s_v(const distance_t &s0, const distance_t &v0, const double &a, const double &t);

/**
 * @brief calculates acceleration for given points and speeds
 */
double acceleration_between(const path_node_t &a, const path_node_t &b);

path_node_t final_velocity_for_accel(const path_node_t &a, const path_node_t &b, const double acceleration);


bool operator==(const path_node_t &lhs,const path_node_t &rhs);

} // namespace physics

} // namespace movement
} // namespace raspigcd

#endif
