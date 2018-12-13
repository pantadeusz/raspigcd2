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

TEST_CASE("gcode_interpreter_test - gcode_to_maps_of_arguments", "[gcd][gcode_interpreter][gcode_to_maps_of_arguments]")
{
    
    SECTION("empty string gives empty map")
    {
        std::list<std::map<char,double> > ret_list = gcode_to_maps_of_arguments("");
        REQUIRE(ret_list.size() == 0);
    }

    SECTION("simple command without enter should result in correct gcode interpretation")
    {
        std::list<std::map<char,double> > ret_list = gcode_to_maps_of_arguments("G0 x 10 ; Y - 20.5");
        REQUIRE(ret_list.size() == 1);
        REQUIRE(ret_list.front().size() == 2);
        REQUIRE((int)(ret_list.front().at('G')) == 0);
        REQUIRE((int)(ret_list.front().at('X')) == 10);
    }

    SECTION("multiple lines with commands, comments and empty lines")
    {
        auto ret_list_ = gcode_to_maps_of_arguments("\nG0X10\nG1Y2\n\n; some comment\nG0X-1Y99; test\n\n\n");
        std::vector<std::map<char,double>> ret_vector{ std::make_move_iterator(std::begin(ret_list_)), std::make_move_iterator(std::end(ret_list_)) };
        REQUIRE(ret_vector.size() == 3);
        
        REQUIRE((int)ret_vector.at(0).size() == 2);
        REQUIRE((int)(ret_vector.at(0).at('G')) == 0);
        REQUIRE((double)(ret_vector.at(0).at('X')) == 10);

        REQUIRE((int)ret_vector.at(1).size() == 2);
        REQUIRE((int)(ret_vector.at(1).at('G')) == 1);
        REQUIRE((double)(ret_vector.at(1).at('Y')) == 2);

        REQUIRE((int)ret_vector.at(2).size() == 3);
        REQUIRE((int)(ret_vector.at(2).at('G')) == 0);
        REQUIRE((double)(ret_vector.at(2).at('X')) == Approx(-1));
        REQUIRE((double)(ret_vector.at(2).at('Y')) == Approx(99));
    }

}