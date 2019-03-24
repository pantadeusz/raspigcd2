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
#include <future>

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



distance_t bezier_rec(const std::vector<distance_t> &points, const double t, const int r, const int i) { 
    // Casteljau algorithm
    if(r == 0) return points[i];
    return (bezier_rec(points, t, r - 1, i) * (1 - t))  + (bezier_rec(points, t, r - 1, i + 1)*t);
};
distance_t bezier_b(const std::vector<distance_t> &points, double t) { 
    return bezier_rec(points, t, points.size()-1, 0);
}


distance_t bezier_rec_a(const std::vector<distance_t> &points, const double t, const int r, const int i) { 
    // Casteljau algorithm
    if(r == 0) return points[i];
    return (bezier_rec_a(points, t, r - 1, i) * (1 - t))  + (bezier_rec_a(points, t, r - 1, i + 1)*t);
};

distance_t bezier(const std::vector<distance_t> &points, const double t) {
    if (points.size() == 1) return points[0];
    if (points.size() < 20) return bezier_rec_a(points, t, points.size()-1, 0);
    auto r = points.size()-1;
    auto i = 0;
    auto left = std::async([&](){return (bezier_rec_a(points, t, r - 1, i) * (1 - t));});
    auto right = std::async([&](){return (bezier_rec_a(points, t, r - 1, i + 1)*t);});
    return left.get() + right.get();
}



std::vector<distance_t> turn_to_bezier(std::vector<distance_t> abc) {
    //auto ab = abc[1]-abc[0];
    //auto bc = abc[2]-abc[1];
    //auto abl = ab.length();
    //auto bcl = bc.length();
    //if (std::abs(abl-bcl) > 0.00000000001) throw std::invalid_argument("the turn must be performed on the equal size arms");
//
    auto nb1 = abc[1];//+ab*1.3;
    auto nb2 = abc[1];//-bc*0.3;
    //nb1[3] = abc[0][3];
    //nb2[3] = abc[2][3];
    return {abc[0],nb1,nb2,abc[2]};
    //auto mp = 
    //return {abc[0],abc[1],abc[1],abc[2]};
}

void experimental_draw_curve_GOOD()
{
    std::vector<distance_t> p0;

    double alpha = 2.0;
    p0.push_back({0, 0, 0, 10});
    p0.push_back({50, 0, 0, 1});
    //p0.push_back({10.0*sin(alpha), 10.0*cos(alpha), 0, 1});
    p0.push_back({100.0,0.0, 0, 1});
    p0.push_back({100.0,50.0, 0, 1});
    p0.push_back({100.0, 100.0, 0, 10});
    //std::vector<distance_t> p;// = p0;//= turn_to_bezier(p0);
    std::vector<distance_t> p = p0;


//std::default_random_engine generator;
//std::uniform_int_distribution<int> distribution(-101,101);
//for (int i = 0; i < 100; i++) distribution(generator);
//    for (int i = 0; i < 13; i++) p.push_back({distribution(generator),distribution(generator),0.0,1.0});
//    p.push_back(p.front());
//    //double t = 0.5;
    
    
    double dt = 1;
    double sum_l = 0;
    for (int i = 1; i < p.size();i++) sum_l += (p[i]-p[i-1]).length();
     for (double t = 0; t <= 1.0;) 
     //for (double t = 0.0; t <= 1.00000000001; t += dt)
     {
         auto bc = bezier(p, t);
         for (auto e : bc)
             std::cout << e << " ";
         std::cout << std::endl;
         t+= dt*bc[3]/sum_l;
     }

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


void experimental_draw_curve(double v0 = 1.0, double dt0 = 0.001)
{

    std::vector<distance_t> p;
    //double t = 0.5;
    p.push_back({0, 0, 0, 0});
    p.push_back({9, 0, 0, 0});
    p.push_back({10, 1, 0, 0});
    //p.push_back({10, 0, 0, 0});
    p.push_back({10, 10, 0, 0});

    double s0 = dt0*v0; // mimiic speed
    double dt = 0.5;
    double dtr = 0.5;
    double sr; 
    do  {
        sr = (bezier(p, 0) - bezier(p, dt)).length();
        dtr *= 0.5;
        if ((s0 - sr) > 0) {
            dt += dtr;
        } else if ((s0 - sr) < 0) {
            dt -= dtr;
        } else break;
    } while (std::abs(s0 - sr) > 0.000001);
    if (dt > 1) dt = 1;
    if (dt < 0) dt = 0.000001;
    for (double t = 0.0; t <= 1.0; t += dt)
    {
        for (auto e : bezier(p, t))
            std::cout << e << " ";
        std::cout << std::endl;
    }
}


} // namespace raspigcd
