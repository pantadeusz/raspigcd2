#include <configuration.hpp>
#include <configuration_json.hpp>
#include <hardware/stepping.hpp>
#include <movement/variable_speed.hpp>
#include <hardware/driver/inmem.hpp>

#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>

#include <chrono>
#include <thread>
#include <vector>

using namespace raspigcd;
using namespace raspigcd::movement;
using namespace raspigcd::configuration;

TEST_CASE( "movement variable speed calculate_linear_coefficient_from_limits", "[movement][calculate_linear_coefficient_from_limits][todo]" ) {
	const std::vector<double> limits_for_axes = {1,2,3,4};
	
	SECTION("1 on each axis") {
		double expected = 1;
		for (const distance_t norm_vect : std::vector<distance_t>{{1,0,0,0},{0,1,0,0},	{0,0,1,0}, {0,0,0,1}}) {
			double result = calculate_linear_coefficient_from_limits(limits_for_axes, norm_vect);
			REQUIRE(result == Approx(expected));
			expected++;
		}
	}
}
