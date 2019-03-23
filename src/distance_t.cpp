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

#include <numeric>

namespace raspigcd {

template<>
double generic_position_t<double>::sumv() const
{
    return std::accumulate(this->begin(), this->end(), 0.0);
}

template<>
double generic_position_t<int>::sumv() const
{
    return std::accumulate(this->begin(), this->end(), 0);
}



namespace bezierhelper {
inline int silnia (int n) {
    static std::vector<int> s = {1,1};
    int size = s.size();
    if (n < size) return s[n];
    if (n > 1) {
        int ret = silnia(n-1)*n;
        if (n == size) s.push_back(ret);
        return ret;
    }
    return 1;
}

int binominal(int n,int k) {
    return silnia(n)/(silnia(k)*silnia(n-k));
}
}
inline distance_t bezier(std::vector<distance_t> p, double t)
{
    using namespace bezierhelper;
    
    distance_t ret(0,0,0,0);
    int n = p.size()-1;
    for (int i = 0; i <= n; i++) {
        ret = ret + p[i]*std::pow((1-t),n-i)*std::pow(t,i)*binominal(n,i);   
    }
    return ret;    
}

void experimental_draw_curve(double dt = 0.01)
{
    std::vector<distance_t> p;
    //double t = 0.5;
    p.push_back({0, 0, 0, 0});
    p.push_back({10, 0, 0, 0});
    p.push_back({10, 10, 0, 0});
    //double dbt = 1.0 / ((p[1]-p[0]).length() + (p[1]-p[2]).length());
    double dbt = dt / ((p[1]-p[0]).length() + (p[1]-p[2]).length());
    for (double t = 0.0; t <= 1.0; t += dbt)
    {
        for (auto e : bezier(p, t))
            std::cout << e << " ";
        std::cout << std::endl;
    }
}




} // namespace raspigcd
