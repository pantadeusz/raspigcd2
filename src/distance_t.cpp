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

template<class T, std::size_t N>
T generic_position_t<T,N>::sumv() const
{
    return std::accumulate(this->begin(), this->end(), 0.0);
}

template<std::size_t N>
void beizer_spline(std::vector<generic_position_t<double,N>> &path,
                   std::function<void(const generic_position_t<double,N> &position)> on_point,
                   double dt, double arc_l) {
  std::vector<std::vector<generic_position_t<double,N>>> triss;
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
  std::list<generic_position_t<double,N>> bezier_points;
  for (unsigned i = 0; i < triss.size(); i++) {
    std::vector<generic_position_t<double,N>> &p = triss[i];

    if (p.size() > 4)
      p.resize(4);

    generic_position_t<double,N> pt;
    double l = 0.000001;
    for (unsigned i = 1; i < p.size(); i++) {
      l += (p[i - 1] - p[i]).length();
    }

    std::cerr << "drawing spline of " << p.size() << ": ";
    for (auto e : p) {
      std::cerr << "[";
      for (auto x : e) {
        std::cerr << x << ",";
      }
      std::cerr << "] ";
    }
    std::cerr << std::endl;
    double dt_p = dt / l;
    if (dt_p < 0.0001) dt_p = 0.0001;
    for (; t <= 1.0;) {
      bezier_points.push_back(pt = bezier(p, t));
      if (bezier_points.size() > 1024*1024*128) {
        std::cerr << "beizer_spline: bezier_points too big: " << bezier_points.size() << " dt_p=" << dt_p << "; t=" << t << std::endl;
        throw std::bad_alloc();
      }
      //double range = std::max(1.0 - t, 0.1) / 2.0;
      //double tp = t + range;
      t += dt_p; //*pt.back()
    }
    t = t - 1.0;
    std::cerr << "t1 " << t << std::endl;
  }
  if (bezier_points.size() > 0) {
    double curr_dist = 0.0;
    std::vector<generic_position_t<double,N>> bcurve(bezier_points.begin(), bezier_points.end());
    bezier_points.clear();
    auto pos = bcurve.front();
    for (unsigned i = 0; i < bcurve.size();) {
      if (bcurve[i].back() < 0.01) {
        std::cerr << "beizer_spline: velocity too small " << bcurve[i] << std::endl;
        bcurve[i].back() = 0.01;
      }
      double target_dist = bcurve[i].back() * dt;
      generic_position_t<double,N> ndistv = bcurve[i] - pos;
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

std::vector <distance_with_velocity_t> optimize_path_dp(std::vector <distance_with_velocity_t> &path, double epsilon) {
    std::vector<char> toDelete(path.size());
    for (auto& e : toDelete)
        e = false;
    //DouglasPeucker algorithm
    std::function<void(double, int, int)> optimizePathDP = [&](double epsilon, int start, int end) {
        double dmax = 0;
        int index = 0;
        for (int i = start + 1; i < end; i++) {
            if (!toDelete[i]) {
                auto d = point_segment_distance_3d(path[i], path[start], path[end]);
                if (d > dmax) {
                    index = i;
                    dmax = d;
                }
            }
        }
        if (dmax > epsilon) {
            optimizePathDP(epsilon, start, index);
            optimizePathDP(epsilon, index, end);
        } else {
            if (start == end)
                return;
            else {
                for (int i = start + 1; i < end; i++) {
                    toDelete[i] = true;
                }
            }
        }
    };
    optimizePathDP(epsilon, 0, path.size() - 1);
    std::vector <distance_with_velocity_t> ret;
    ret.reserve(path.size());
    for (int i = 0; i < path.size();i++) {
        if (!toDelete[i]) ret.push_back(path[i]);
    }
    ret.shrink_to_fit();
    return ret;
}

/// instantiate templates

template int generic_position_t<int,5>::sumv() const;
template int generic_position_t<int,4>::sumv() const;
template int generic_position_t<int,3>::sumv() const;
template int generic_position_t<int,2>::sumv() const;

template double generic_position_t<double,5>::sumv() const;
template double generic_position_t<double,4>::sumv() const;
template double generic_position_t<double,3>::sumv() const;
template double generic_position_t<double,2>::sumv() const;



template void beizer_spline<2>(std::vector<generic_position_t<double,2>> &path,
                   std::function<void(const generic_position_t<double,2> &position)> on_point,
                   double dt, double arc_l);


template void beizer_spline<3>(std::vector<generic_position_t<double,3>> &path,
                   std::function<void(const generic_position_t<double,3> &position)> on_point,
                   double dt, double arc_l);

template void beizer_spline<4>(std::vector<generic_position_t<double,4>> &path,
                   std::function<void(const generic_position_t<double,4> &position)> on_point,
                   double dt, double arc_l);

template void beizer_spline<5>(std::vector<generic_position_t<double,5>> &path,
                   std::function<void(const generic_position_t<double,5> &position)> on_point,
                   double dt, double arc_l);


} // namespace raspigcd
