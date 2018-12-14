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

TEST_CASE("gcode_interpreter_test - generate_path_intent", "[gcd][gcode_interpreter][generate_path_intent]")
{
    auto to_vector = [](const auto &ret_list_){
        return std::vector< movement::path_intent_element_t >{ 
            std::begin(ret_list_),
            std::end(ret_list_)
            };
    };
    SECTION("empty gives empty")
    {
        auto ret = generate_path_intent({});
        REQUIRE(ret.size() == 0);
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
