/*
    Raspberry Pi G-CODE interpreter
    Copyright (C) 2018  Tadeusz Puźniakowski puzniakowski.pl
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.
    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef __RASPIGCD_DISTANCE_T_HPP__
#define __RASPIGCD_DISTANCE_T_HPP__

#include <array>

namespace raspigcd {

class distance_t : public std::array<double, 4> {
public:
    double &a(){return this->operator[](0);};
    double &b(){return this->operator[](1);};
    double &c(){return this->operator[](2);};
    double &d(){return this->operator[](3);};
    distance_t() : std::array<double, 4>() {};
    distance_t(double a_, double b_, double c_, double d_) : std::array<double, 4>() {
        a() = a_;
        b() = b_;
        c() = c_;
        d() = d_;
    };
};
}
#endif
