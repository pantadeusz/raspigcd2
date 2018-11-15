#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>
#include <gcd/path_intent_executor.hpp>
#include <chrono>
#include <thread>
#include <vector>

using namespace raspigcd;
using namespace raspigcd::movement;
using namespace raspigcd::configuration;

TEST_CASE( "path_intent_executor", "[gcd][path_intent_executor]" ) {
	
	SECTION("simple program executor") {

	}
}
