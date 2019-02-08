#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>


#include "tests_helper.hpp"
#include <gcd/gcode_interpreter.hpp>

using namespace raspigcd;
using namespace raspigcd::configuration;
using namespace raspigcd::gcd;

std::vector<std::vector<int>> simulate_moves_on_image(const program_t& prg, const block_t& initial_state)
{
    auto state = merge_blocks(initial_state, {{'X', 0}, {'Y', 0}, {'Z', 0}, {'A', 0}});
    std::pair<double, double> range_x = {state['X'], state['X']};
    std::pair<double, double> range_y = {state['Y'], state['Y']};
    for (auto s : prg) {
        range_x.first = std::min(range_x.first, state['X']);
        range_x.second = std::max(range_x.second, state['X']);
        range_y.first = std::min(range_y.first, state['Y']);
        range_y.second = std::max(range_y.second, state['Y']);
        state = merge_blocks(state, s);
        range_x.first = std::min(range_x.first, state['X']);
        range_x.second = std::max(range_x.second, state['X']);
        range_y.first = std::min(range_y.first, state['Y']);
        range_y.second = std::max(range_y.second, state['Y']);
    }
    range_x.first *= 10;
    range_x.second *= 10;range_x.second += 11;
    range_y.first *= 10;
    range_y.second *= 10;range_y.second +=11;
    std::vector<std::vector<int>> ret(range_y.second - range_y.first,
        std::vector<int>(range_x.second - range_x.first, 0));
    state = merge_blocks(initial_state, {{'X', 0}, {'Y', 0}, {'Z', 0}, {'A', 0}});

    for (auto stt : prg) {
        auto new_state = merge_blocks(state, stt);

        distance_t p0 = block_to_distance_t(state);
        distance_t p1 = block_to_distance_t(new_state);
    
    
        double v0 = 1;
        distance_t vp = p1 - p0;
        double s = std::sqrt(vp.length2()); ///< summary distance to go
        distance_t vp_v = vp / s;

        //double T = s / v0;
        double a = 0;
        for (int i = 0; i < s*10; ++i) {
            auto pos = p0 + vp_v * i;
            ret.at(pos[1]).at(pos[0]) = pos[2];
        }
        state = new_state;
    }
    return ret;
}

int image_difference(const std::vector<std::vector<int>>& a, const std::vector<std::vector<int>>& b)
{
    int s = 0;
    for (int y = a.size(); y--;) {
        for (int x = a[y].size(); x--;) {
            if ((a.at(y).at(x) - b.at(y).at(x)) != 0) {
                std::cout << "diff at: " 
                << x << " " << y 
                << ":: " << a.at(y).at(x) << "  ?  " <<  b.at(y).at(x) << " :: "
                << std::endl;
            }
            s += a.at(y).at(x) - b.at(y).at(x);
        }
    }
    return s;
}

