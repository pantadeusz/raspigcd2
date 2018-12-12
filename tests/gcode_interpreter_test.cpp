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

TEST_CASE("gcode_interpreter_test", "[gcd][gcode_interpreter]")
{
    
    SECTION("empty string gives empty map")
    {
        std::map<char,double> ret = command_to_map_of_arguments("");
        REQUIRE(ret.size() == 0);
    }
    SECTION("whitespace string gives empty result")
    {
        std::map<char,double> ret = command_to_map_of_arguments(" \t\r");
        REQUIRE(ret.size() == 0);
    }
    SECTION("new line gives exception")
    {
        REQUIRE_THROWS( command_to_map_of_arguments(" \t\r\n "));
    }
    SECTION("new line gives invalid argument exception")
    {
        REQUIRE_THROWS_AS( command_to_map_of_arguments(" \t\r\n "),std::invalid_argument);
        REQUIRE_THROWS_WITH( command_to_map_of_arguments(" \t\r\n "),"new line is not allowed");
    }
    SECTION("One character and one int value")
    {
        std::map<char,double> ret = command_to_map_of_arguments("G0");
        REQUIRE(ret.size() == 1);
        REQUIRE((int)(ret.at('G')) == 0);
    }
    SECTION("Case insensitive multiple commands")
    {
        std::map<char,double> ret = command_to_map_of_arguments("G0x10Y20");
        REQUIRE(ret.size() == 3);
        REQUIRE((int)(ret.at('G')) == 0);
        REQUIRE((int)(ret.at('X')) == 10);
        REQUIRE((int)(ret.at('Y')) == 20);
    }
    SECTION("Spaces in data")
    {
        std::map<char,double> ret = command_to_map_of_arguments("G0 x 10 Y - 20.5");
        REQUIRE(ret.size() == 3);
        REQUIRE((int)(ret.at('G')) == 0);
        REQUIRE((int)(ret.at('X')) == 10);
        REQUIRE(ret.at('Y') == Approx(-20.5));
    }

    SECTION("Comments should be ignored")
    {
        std::map<char,double> ret = command_to_map_of_arguments("G0 x 10 ; Y - 20.5");
        REQUIRE(ret.size() == 2);
        REQUIRE((int)(ret.at('G')) == 0);
        REQUIRE((int)(ret.at('X')) == 10);
    }
    
}

