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



#ifndef __RASPIGCD_generic_position_t_HPP__
#define __RASPIGCD_generic_position_t_HPP__

#include <array>
#include <vector>
#include <iostream>
#include <cassert>
#include <cmath>

#include <hardware_dof_conf.hpp>

namespace raspigcd {

template<class T>
class generic_position_t;

template<class T>
inline generic_position_t<T> operator*(const generic_position_t<T>& a, const generic_position_t<T>& b);

/**
 * The class that represents distances between points in euclidean space.
 * 
 * You can also call it coordinates if you want.
 * */

template<class T>
class generic_position_t : public std::array<T, RASPIGCD_HARDWARE_DOF>
{
public:
    T& a() { return this->operator[](0); };
    T& b() { return this->operator[](1); };
    T& c() { return this->operator[](2); };
    T& d() { return this->operator[](3); };
    generic_position_t() : std::array<T, RASPIGCD_HARDWARE_DOF>(){};
    generic_position_t(const double &a_, const double &b_, const double &c_, const double &d_) 
    : std::array<T, RASPIGCD_HARDWARE_DOF>()
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
    
    double angle(const generic_position_t & a, const generic_position_t & b) const;

    double sumv() const;

    inline double dot_product(const generic_position_t &b) const {
        const generic_position_t a = *this;
        return (a * b).sumv();
    }
};

template<class T>
inline generic_position_t<T> operator+(const generic_position_t<T>& a, const generic_position_t<T>& b)
{
    return generic_position_t<T>(
        a[0] + b[0],
        a[1] + b[1],
        a[2] + b[2],
        a[3] + b[3]);
}

template<class T>
inline generic_position_t<T> operator-(const generic_position_t<T>& a, const generic_position_t<T>& b)
{
    return generic_position_t<T>(
        a[0] - b[0],
        a[1] - b[1],
        a[2] - b[2],
        a[3] - b[3]);
}

template<class T>
inline generic_position_t<T> operator*(const generic_position_t<T>& a, const generic_position_t<T>& b)
{
    return generic_position_t<T>(
        a[0] * b[0],
        a[1] * b[1],
        a[2] * b[2],
        a[3] * b[3]);
}

template<class T>
inline generic_position_t<T> operator/(const generic_position_t<T>& a, const generic_position_t<T>& b)
{
    return generic_position_t<T>(
        a[0] / b[0],
        a[1] / b[1],
        a[2] / b[2],
        a[3] / b[3]);
}

template<class T>
inline generic_position_t<T> operator*(const generic_position_t<T>& a, const double& b)
{
    return generic_position_t<T>(
        a[0] * b,
        a[1] * b,
        a[2] * b,
        a[3] * b);
}

template<class T>
inline generic_position_t<T> operator/(const generic_position_t<T>& a, const double& b)
{
    return generic_position_t<T>(
        a[0] / b,
        a[1] / b,
        a[2] / b,
        a[3] / b);
}

template<class T>
inline double generic_position_t<T>::angle(const generic_position_t<T>& a, const generic_position_t<T>& b) const
{
    auto u = a - (*this);
    auto v = b - (*this);
    auto dotprod = (u * v).sumv();
    if (dotprod == 0) return 3.14159265358979323846 / 2.0;
    return acos(dotprod / (std::sqrt(u.length2()) * std::sqrt(v.length2())));
}

template<class T>
inline bool operator==(const generic_position_t<T>& a, const generic_position_t<T>& b)
{
    return (a[0] == b[0]) &&
           (a[1] == b[1]) &&
           (a[2] == b[2]) &&
           (a[3] == b[3]);
}

template<class T>
inline std::ostream& operator<<(std::ostream& os, generic_position_t<T> const& value)
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
//double calculate_linear_coefficient_from_limits(const std::vector<double>& limits_for_axes, const generic_position_t& norm_vect);

//double calculate_linear_coefficient_from_limits(const std::vector<double>& limits_for_axes, const generic_position_t& norm_vect)
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

using distance_t = generic_position_t<double>;



distance_t bezier(const std::vector<distance_t> &p, const double t);


} // namespace raspigcd
#endif
