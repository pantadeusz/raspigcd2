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


#ifndef __RASPIGCD_DISTANCE_T_HPP__
#define __RASPIGCD_DISTANCE_T_HPP__

#include <array>
#include <iostream>

namespace raspigcd {

class distance_t : public std::array<double, 4>
{
public:
    double& a() { return this->operator[](0); };
    double& b() { return this->operator[](1); };
    double& c() { return this->operator[](2); };
    double& d() { return this->operator[](3); };
    distance_t() : std::array<double, 4>(){};
    distance_t(const double a_, const double b_, const double c_, const double d_) : std::array<double, 4>()
    {
        a() = a_;
        b() = b_;
        c() = c_;
        d() = d_;
    };
    inline double length2() const
    {
        return this->operator[](0) * this->operator[](0) +
               this->operator[](1) * this->operator[](1) +
               this->operator[](2) * this->operator[](2) +
               this->operator[](3) * this->operator[](3);
    }
    inline double sumv() const
    {
        return this->operator[](0)+
               this->operator[](1)+
               this->operator[](2)+
               this->operator[](3);
    }
};

inline distance_t operator+(const distance_t& a, const distance_t& b)
{
    return distance_t(
        a[0] + b[0],
        a[1] + b[1],
        a[2] + b[2],
        a[3] + b[3]);
}
inline distance_t operator-(const distance_t& a, const distance_t& b)
{
    return distance_t(
        a[0] - b[0],
        a[1] - b[1],
        a[2] - b[2],
        a[3] - b[3]);
}
inline distance_t operator*(const distance_t& a, const distance_t& b)
{
    return distance_t(
        a[0] * b[0],
        a[1] * b[1],
        a[2] * b[2],
        a[3] * b[3]);
}
inline distance_t operator/(const distance_t& a, const distance_t& b)
{
    return distance_t(
        a[0] / b[0],
        a[1] / b[1],
        a[2] / b[2],
        a[3] / b[3]);
}
inline distance_t operator*(const distance_t& a, const double& b)
{
    return distance_t(
        a[0] * b,
        a[1] * b,
        a[2] * b,
        a[3] * b);
}
inline distance_t operator/(const distance_t& a, const double& b)
{
    return distance_t(
        a[0] / b,
        a[1] / b,
        a[2] / b,
        a[3] / b);
}

inline bool operator==(const distance_t& a, const distance_t& b)
{
    return 
        (a[0] == b[0]) &&
        (a[1] == b[1]) &&
        (a[2] == b[2]) &&
        (a[3] == b[3]);
}

inline std::ostream &operator<<(std::ostream &os, distance_t const &value) {
    os << "[" << value[0] << ", " << value[1] << ", " << value[2] << ", " << value[3] << "]";
    return os;
}

} // namespace raspigcd
#endif
