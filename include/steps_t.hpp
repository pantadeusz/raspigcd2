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

#ifndef __RASPIGCD_STEPS_T_HPP__
#define __RASPIGCD_STEPS_T_HPP__

#include <array>

namespace raspigcd {

class steps_t : public std::array<int, 4> {
public:
    inline int &a(){return this->operator[](0);};
    inline int &b(){return this->operator[](1);};
    inline int &c(){return this->operator[](2);};
    inline int &d(){return this->operator[](3);};
    steps_t() : std::array<int, 4>() {};
    steps_t(int a_, int b_, int c_, int d_) : std::array<int, 4>() {
        a() = a_;
        b() = b_;
        c() = c_;
        d() = d_;
    };
    steps_t(const int *s_) : std::array<int, 4>() {
        a() = s_[0];
        b() = s_[1];
        c() = s_[2];
        d() = s_[3];
    };
};

inline bool operator==(const steps_t &l, const steps_t &r) {
    for (unsigned i = 0; i < l.size(); i++ ) if (l[i] != r[i]) return false;
    return true;
}
inline bool operator!=(const steps_t &l, const steps_t &r) {
    return !(l == r);
}

inline steps_t operator+(const steps_t &l, const steps_t &r) {
    return {
        l[0]+r[0],
        l[1]+r[1],
        l[2]+r[2],
        l[3]+r[3],
        };
}
inline steps_t operator-(const steps_t &l, const steps_t &r) {
    return {
        l[0]-r[0],
        l[1]-r[1],
        l[2]-r[2],
        l[3]-r[3],
        };
}

}
#endif
