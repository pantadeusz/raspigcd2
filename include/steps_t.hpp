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
}
#endif
