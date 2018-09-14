#ifndef __RASPIGCD_DISTANCE_T_HPP__
#define __RASPIGCD_DISTANCE_T_HPP__

#include <array>

namespace raspigcd {

class distance_t : public std::array<double, 4> {
public:
    double &a(){return this->operator[](0);};
    double &b(){return this->operator[](1);};
    double &c(){return this->operator[](2);};
    double &d(){return this->operator[](3);};
    distance_t() : std::array<double, 4>() {};
    distance_t(double a_, double b_, double c_, double d_) : std::array<double, 4>() {
        a() = a_;
        b() = b_;
        c() = c_;
        d() = d_;
    };
};
}
#endif
