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




#include <array>
#include <cassert>
#include <cmath>
#include <iostream>

#include <distance_t.hpp>

namespace raspigcd {
double distance_t::angle(const distance_t& a, const distance_t& b) const
{
    auto u = a - (*this);
    auto v = b - (*this);
    auto dotprod = (u * v).sumv();
    if (dotprod == 0) return 3.14159265358979323846 / 2.0;
    return acos(dotprod / (std::sqrt(u.length2()) * std::sqrt(v.length2())));
}

} // namespace raspigcd
