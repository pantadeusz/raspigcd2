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

#include <configuration.hpp>
#include <distance_t.hpp>
#include <movement/physics.hpp>
//#include <hardware/stepping_commands.hpp>
#include <list>
#include <steps_t.hpp>
#include <cmath>

namespace raspigcd {
namespace movement {

/**
 * @brief the physics library that can be used to calculate accelerations, distances and other things.
 */
namespace physics {

distance_t get_next_position(const distance_t &s0, const distance_t &v0, const double &a, const double &t) {
    auto l = std::sqrt(v0.length2());
    if (l == 0.0) return s0;
    distance_t a_= (v0/l)*a;
    distance_t s1 = s0 + v0 * t + a_*t*t/2.0;
    return s1;
}

distance_t get_next_velocity(const distance_t &s0, const distance_t &v0, const double &a, const double &t) {
    auto l = std::sqrt(v0.length2());
    if (l == 0.0) return v0;
    distance_t a_= (v0/l)*a;
    return v0 + a_*t;
}

path_node_t get_next_node(const distance_t &s0, const distance_t &v0, const double &a, const double &t) {
    auto l = std::sqrt(v0.length2());
    if (l == 0.0) return {s0,std::sqrt(v0.length2())};

    const auto& [s1,v1] = get_next_s_v(s0, v0, a,t);

    auto v0s = std::sqrt((v0).length2());
    auto v1s = std::sqrt((v1).length2());
    if (v1s == 0.0) return {s1,0.0};
    auto v0norm = v0/v0s;
    auto v1norm = v1/v1s;
    auto vdiff = (int)std::sqrt((v1norm-v0norm).length2());
    double d = (1.0 - vdiff);
    return {s1,(d*v1s)};
}

std::pair<distance_t,distance_t> get_next_s_v(const distance_t &s0, const distance_t &v0, const double &a, const double &t) {
    auto l = std::sqrt(v0.length2());
    if (l == 0.0) return {s0,v0};
    distance_t a_= (v0/l)*a;
    distance_t s1 = s0 + v0 * t + a_*t*t/2.0;
    auto v1 = v0 + a_*t;
    
    return {s1,v1};
}



double acceleration_between(const path_node_t &a, const path_node_t &b) {
    auto s = (b.p-a.p).length();
    auto dv = (b.v-a.v);
    if (dv == 0) return 0;
    if (s == 0) throw std::invalid_argument("cannot do infinite accelerations");
    
    double a_min = -10000000.0, a_max= 10000000.0;
    double tt = 0;
    for (int n = 0; n < 82; n++) {
        double acc = (a_max + a_min)/2.0;
        double t = 0.0;
        if (acc != 0.0) {
            t = (b.v-a.v)/acc;
            tt = t;
            double s1 = a.v * t + acc*t*t/2.0;
            
            if (s1 > s) {
                a_min = acc;
            } else if (s1 < s) {
                a_max = acc;
            } else if (s1 == s) {
                //std::cout << "********n = " << n << std::endl;
                return (a_max + a_min)/2.0;
            }
        } else if ((b.v-a.v) > 0) {
            a_min = acc;
        } else if ((b.v-a.v) > 0) {
            a_max = acc;
        } else {
            //std::cout << "********acc = " << 0 << std::endl;
            return 0.0;
        }
    }
    std::cout << "WARNING: not precise result: double acceleration_between(const path_node_t &a, const path_node_t &b) : s: " << s <<" dv " << dv << "    tt " << tt << std::endl;
    return (a_max + a_min)/2.0;
}

} // namespace physics

} // namespace movement
} // namespace raspigcd

