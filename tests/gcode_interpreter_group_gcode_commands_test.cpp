#define CATCH_CONFIG_MAIN
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

using namespace raspigcd;
using namespace raspigcd::configuration;
using namespace raspigcd::gcd;

TEST_CASE("gcode_interpreter_test - group_gcode_commands", "[gcd][gcode_interpreter][group_gcode_commands]")
{
    auto simple_gcode_one_group_g1 = gcode_to_maps_of_arguments(R"(
        G1X10
        G1Y10
        G1X0
        G1Y0
    )");
    auto simple_gcode_five_groups_mg1g0g1m = gcode_to_maps_of_arguments(R"(
        M17
        M3
        G1Z5
        G1X10
        G1Y10
        G1X0
        G1Y0
        G0X10Y10
        G1X20Y30
        G1X0
        G1Y0
        G1Z0
        M5
        M18
    )");
    


    SECTION("empty gives empty")
    {
        auto ret = group_gcode_commands({});
        REQUIRE(ret.size() == 0);
    }
    SECTION("Single group simple_gcode_one_group_g1")
    {
        auto ret = group_gcode_commands(simple_gcode_one_group_g1);
        REQUIRE(ret.size() == 1);
    }
    SECTION("simple_gcode_five_groups_mg1g0g1m shoulod give five groups")
    {
        auto ret = group_gcode_commands(simple_gcode_five_groups_mg1g0g1m);
        REQUIRE(ret.size() == 5);
    }

    SECTION("Single group simple_gcode_one_group_g1 should result in additional command at start")
    {
        auto ret = group_gcode_commands(simple_gcode_one_group_g1,{{'F',3}});
        //INFO(back_to_gcode(ret));
        REQUIRE(ret.size() == 1);
        REQUIRE(ret.front().size() == 5);
        REQUIRE(ret.front().front().at('F') == Approx(3));
        REQUIRE(ret.front().front().at('G') == Approx(1));
    }

//     SECTION("simple G0 command with movement along X")
//     {
//         auto ret = to_vector(generate_path_intent({
//             { {'G',0.0}, {'X',10.0} }
//         }));
// 
//         REQUIRE(ret.size() == 3);
//         REQUIRE(std::get<distance_t>(ret.at(0)) == distance_t{0.0,0.0,0.0,0.0});
//         REQUIRE(std::get<movement::path_intentions::move_t>(ret.at(1)) == Approx(100));
//         REQUIRE(std::get<distance_t>(ret.at(2)) == distance_t{10.0,0.0,0.0,0.0});
//     }

}
