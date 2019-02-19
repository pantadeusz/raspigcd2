/*
    Raspberry Pi G-CODE interpreter

    Copyright (C) 2019  Tadeusz Puźniakowski puzniakowski.pl

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
#include <iostream>
#include <cassert>
#include <cmath>

#include <hardware_dof_conf.hpp>

namespace raspigcd {

/**
 * The class that represents distances between points in euclidean space.
 * 
 * You can also call it coordinates if you want.
 * */
class distance_t : public std::array<double, RASPIGCD_HARDWARE_DOF>
{
public:
    double& a() { return this->operator[](0); };
    double& b() { return this->operator[](1); };
    double& c() { return this->operator[](2); };
    double& d() { return this->operator[](3); };
    distance_t() : std::array<double, RASPIGCD_HARDWARE_DOF>(){};
    distance_t(const double a_, const double b_, const double c_, const double d_) : std::array<double, RASPIGCD_HARDWARE_DOF>()
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
    inline double length() const
    {
        return std::sqrt(length2());
    }
    
    double angle(const distance_t & a, const distance_t & b) const;

    inline double sumv() const
    {
        return this->operator[](0) +
               this->operator[](1) +
               this->operator[](2) +
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
    return (a[0] == b[0]) &&
           (a[1] == b[1]) &&
           (a[2] == b[2]) &&
           (a[3] == b[3]);
}

inline std::ostream& operator<<(std::ostream& os, distance_t const& value)
{
    os << "[" << value[0] << ", " << value[1] << ", " << value[2] << ", " << value[3] << "]";
    return os;
}


/*
    * calulates maximal linear value given the maximal values for each axis, and the normal vector of intended move
    * it works that if norm_vect is 1 on one axis, then the value from limits_for_axes on this
    * otherwise it blends, so if it is a limit, then applying linear limit will not exceed limits
    * it is used to calculate maximal speed for movement in any direction, as well as maximal acceleration and speed without acceleration
    */
//double calculate_linear_coefficient_from_limits(const std::vector<double>& limits_for_axes, const distance_t& norm_vect);

//double calculate_linear_coefficient_from_limits(const std::vector<double>& limits_for_axes, const distance_t& norm_vect)
//std::vector<double>
auto calculate_linear_coefficient_from_limits = [](const auto& limits_for_axes, const auto& norm_vect) -> double
{
    double average_max_accel = 0;
    double average_max_accel_sum = 0;
    for (unsigned int i = 0; i < limits_for_axes.size(); i++) {
        average_max_accel += limits_for_axes.at(i) * std::abs(norm_vect.at(i));
        average_max_accel_sum += std::abs(norm_vect.at(i));
    }
    average_max_accel = average_max_accel / average_max_accel_sum;
    return average_max_accel;
};


} // namespace raspigcd
#endif
