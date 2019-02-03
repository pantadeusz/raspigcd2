#include <configuration.hpp>
#include <configuration_json.hpp>
#include <hardware/stepping.hpp>
#include <movement/simple_steps.hpp>

#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>

#include <chrono>
#include <thread>
#include <vector>

using namespace raspigcd;
using namespace raspigcd::configuration;
using namespace raspigcd::movement::simple_steps;
using namespace raspigcd::movement;


TEST_CASE("Movement constant speed in steps generator", "[movement][steps_generator]")
{
    SECTION("collapse_repeated_steps")
    {
        auto result = collapse_repeated_steps({});
        REQUIRE(result.size() == 0);

        result = collapse_repeated_steps({{
                                                                .b = {{.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}},
                                                                .count = 1}});
        REQUIRE(result.size() == 1);

        result = collapse_repeated_steps(
            {
            {.b = {{.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}}, .count = 1},
            {.b = {{.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}}, .count = 1}
            }
            );
        REQUIRE(result.size() == 1);
        REQUIRE(result[0].count == 2);

        result = collapse_repeated_steps(
            {
            {.b = {{.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}}, .count = 1},
            {.b = {{.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}}, .count = 0}
            }
            );
        REQUIRE(result.size() == 1);
        REQUIRE(result[0].count == 1);

        result = collapse_repeated_steps(
            {
            {.b = {{.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}}, .count = 0},
            {.b = {{.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}}, .count = 0}
            }
            );
        REQUIRE(result.size() == 0);

        result = collapse_repeated_steps(
            {
            {.b = {{.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}}, .count = 1},
            {.b = {{.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}}, .count = 2}
            }
            );
        REQUIRE(result.size() == 1);
        REQUIRE(result[0].count == 3);

        result = collapse_repeated_steps(
            {
            {.b = {{.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}}, .count = 1},
            {.b = {{.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}}, .count = 2},
            {.b = {{.step = 1, .dir = 0}, {.step = 1, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}}, .count = 2}
            }
            );
        REQUIRE(result.size() == 2);
        REQUIRE(result[0].count == 3);
        REQUIRE(result[1].count == 2);
    }
}
