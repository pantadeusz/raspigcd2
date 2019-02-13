#ifndef __TESTS_HELPER__HPP___
#define __TESTS_HELPER__HPP___

#include <gcd/gcode_interpreter.hpp>

std::vector<std::vector<int>> simulate_moves_on_image(
    const raspigcd::gcd::program_t& prg, const raspigcd::gcd::block_t& initial_state = {});
std::vector<std::vector<int>> simulate_moves_on_image(
    const raspigcd::hardware::multistep_commands_t & prg,
    raspigcd::hardware::motor_layout &motor_layout
    );
int image_difference(const std::vector<std::vector<int>>& a, const std::vector<std::vector<int>>& b);



std::vector<std::vector<int>> load_image(std::string filename);
void save_image(const std::string filename, const std::vector<std::vector<int>> &img_dta);

#endif