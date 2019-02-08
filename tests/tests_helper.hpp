#ifndef __TESTS_HELPER__HPP___
#define __TESTS_HELPER__HPP___

#include <gcd/gcode_interpreter.hpp>

std::vector<std::vector<int>> simulate_moves_on_image(
    const raspigcd::gcd::program_t& prg, const raspigcd::gcd::block_t& initial_state = {});
int image_difference(const std::vector<std::vector<int>>& a, const std::vector<std::vector<int>>& b);

#endif