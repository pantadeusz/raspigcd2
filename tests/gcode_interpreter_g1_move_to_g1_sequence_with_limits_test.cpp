#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>
#include <chrono>
#include <gcd/gcode_interpreter.hpp>
#include <hardware/driver/inmem.hpp>
#include <hardware/driver/low_buttons_fake.hpp>
#include <hardware/driver/low_spindles_pwm_fake.hpp>
#include <hardware/driver/low_timers_fake.hpp>
#include <hardware/stepping.hpp>
#include <thread>
#include <vector>

#include "tests_helper.hpp"

using namespace raspigcd;
using namespace raspigcd::configuration;
using namespace raspigcd::gcd;
//apply_limits_for_turns
TEST_CASE("gcode_interpreter_test - g1_move_to_g1_with_machine_limits - simple cases", "[gcd][gcode_interpreter][g1_move_to_g1_with_machine_limits][exceptions]")
{
    configuration::limits machine_limits(
        {100, 101, 102, 103}, // acceleration
        {50, 51, 52, 53},     // max velocity
        {2, 3, 4, 5});        // no accel velocity

    program_t program = gcode_to_maps_of_arguments("");
    program_t program_not_g0_g1_a = gcode_to_maps_of_arguments("G4X20M10\nM17");
    program_t program_not_g0_g1_b = gcode_to_maps_of_arguments("M17\nM5");
    program_t empty_program = {};

    SECTION("empty program results in exception")
    {
        REQUIRE_THROWS_AS(
            g1_move_to_g1_with_machine_limits(empty_program, machine_limits),
            std::invalid_argument);
        REQUIRE_THROWS_WITH(
            g1_move_to_g1_with_machine_limits(empty_program, machine_limits),
            "there must be at least one G0 or G1 code in the program!");
    }

    SECTION("the program that contains other commands than G0 or G1 should be considered invalid")
    {
        REQUIRE_THROWS_AS(
            g1_move_to_g1_with_machine_limits(program_not_g0_g1_a, machine_limits),
            std::invalid_argument);
        REQUIRE_THROWS_WITH(
            g1_move_to_g1_with_machine_limits(program_not_g0_g1_a, machine_limits),
            "G0 or G1 should be the only type of the commands in the program for g1_move_to_g1_with_machine_limits");
        REQUIRE_THROWS_AS(
            g1_move_to_g1_with_machine_limits(program_not_g0_g1_b, machine_limits),
            std::invalid_argument);
        REQUIRE_THROWS_WITH(
            g1_move_to_g1_with_machine_limits(program_not_g0_g1_b, machine_limits),
            "G0 or G1 should be the only type of the commands in the program for g1_move_to_g1_with_machine_limits");
    }
}

TEST_CASE("gcode_interpreter_test - g1_move_to_g1_with_machine_limits - check if resulting gcode contains correct path", "[gcd][gcode_interpreter][g1_move_to_g1_with_machine_limits][in_limits]")
{
    configuration::limits machine_limits(
        {100, 101, 102, 103}, // acceleration
        {50, 51, 52, 53},     // max velocity
        {2, 3, 4, 5});        // no accel velocity
    program_t program_0 = gcode_to_maps_of_arguments("G1X10F1");

    SECTION("program without any acceleration that must be reduced")
    {
        auto program_0_prim = g1_move_to_g1_with_machine_limits(program_0, machine_limits);
        INFO(back_to_gcode({program_0}));
        INFO(back_to_gcode({program_0_prim}));
        auto img_before = simulate_moves_on_image(program_0);
        auto img_after = simulate_moves_on_image(program_0_prim);
        REQUIRE(image_difference(img_before, img_after) == 0);
    }
}

//TEST_CASE("gcode_interpreter_test - g1_move_to_g1_with_machine_limits - check if resulting gcode is within machine limits", "[gcd][gcode_interpreter][g1_move_to_g1_with_machine_limits][in_limits]")
//{
//
//}
