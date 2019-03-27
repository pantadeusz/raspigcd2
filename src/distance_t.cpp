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
#include <tuple>
#include <list>
#include <vector>
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



/**
 *
 * */
void beizer_spline(std::vector<distance_t> &path,
                   std::function<void(const distance_t &position)> on_point,
                   double dt, double arc_l) {
  std::vector<std::vector<distance_t>> triss;
  if (path.size() < 3) {
    triss.push_back(path);
  } else {
    auto additional_p = [&arc_l](auto &a0, auto &b, auto &c0) {
      auto ba0 = (b - a0);
      auto ba0l = ba0.length();
      auto bc0 = (b - c0);
      auto bc0l = bc0.length();

      auto a = b - ba0 / ba0l;
      auto c = b - bc0 / bc0l;

      auto projv = b - b.projection(a, c);
      auto d = a + projv;
      auto e = c + projv;
      auto vvv = d - e;
      auto vvvl = vvv.length();
      d = b + vvv * arc_l / vvvl;
      e = b - vvv * arc_l / vvvl;
      return std::make_pair(d, e);
    };

    {
      auto a = path[0];
      auto b = path[1];
      auto c = path[2];
      auto [d, e] = additional_p(a, b, c);
      triss.push_back({a, (a + b) / 2.0, d, b, e});
    }
    for (unsigned i = 1; i < path.size() - 2; i++) {
      auto a = path[i];
      auto b = path[i + 1];
      auto c = path[i + 2];
      auto [d, e] = additional_p(a, b, c);
      std::cerr << d << " -> " << e << std::endl;
      triss.push_back({a, triss.back().back(), d, b, e});
    }
    {
      int i = path.size() - 2;
      auto a = path[i];
      auto b = path[i + 1];
      triss.push_back({a, triss.back().back(), b});
    }

    std::cerr << std::endl;
  }

  double t = 0;
  std::list<distance_t> bezier_points;
  for (unsigned i = 0; i < triss.size(); i++) {
    std::vector<distance_t> &p = triss[i];

    if (p.size() > 4)
      p.resize(4);

    std::cerr << "drawing spline of " << p.size() << ": ";
    for (auto e : p) {
      std::cerr << "[";
      for (auto x : e) {
        std::cerr << x << ",";
      }
      std::cerr << "] ";
    }
    std::cerr << std::endl;
    distance_t pt;
    double l = 0.000001;
    for (unsigned i = 1; i < p.size(); i++) {
      l += (p[i - 1] - p[i]).length();
    }
    for (; t <= 1.0;) {
      bezier_points.push_back(pt = bezier(p, t));
      //double range = std::max(1.0 - t, 0.1) / 2.0;
      //double tp = t + range;
      t += 0.1 * dt / l; //*pt.back()
    }
    t = t - 1.0;
    std::cerr << "t1 " << t << std::endl;
  }
  if (bezier_points.size() > 0) {
    double curr_dist = 0.0;
    std::vector<distance_t> bcurve(bezier_points.begin(), bezier_points.end());
    bezier_points.clear();
    auto pos = bcurve.front();
    for (unsigned i = 0; i < bcurve.size();) {
      double target_dist = bcurve[i].back() * dt;
      distance_t ndistv = bcurve[i] - pos;
      ndistv.back() = 0;
      double ndist = ndistv.length();
      if ((curr_dist + ndist) >= target_dist) {
        auto mv = (ndistv / ndist) * (target_dist - curr_dist);
        pos = pos + mv;
        on_point(pos);
        curr_dist = 0.0;
      } else {
        curr_dist += ndist;
        pos = bcurve[i];
        i++;
      }
    }
    //        on_point(pos);
  }
}

} // namespace raspigcd
