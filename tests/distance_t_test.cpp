#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>
#include <distance_t.hpp>
#include <thread>
#include <vector>

using namespace raspigcd;

TEST_CASE("distance_t - check vector operations", "[common][distance_t]")
{
    SECTION("when two points are the same relative to the distance_t, then the angle should be 0")
    {
        distance_t middle = {100,200,150,50};
        distance_t a = {130,200,150,50};
        distance_t b = {130,200,150,50};
        
        auto ret = middle.angle(a,b);
        REQUIRE(ret == 0);
    }
    SECTION("when two points are the same line relative to the distance_t, then the angle should be M_PI")
    {
        distance_t middle = {100,200,150,50};
        distance_t a = {100+10,200+20,150+5 ,50+1};
        distance_t b = {100-10,200-20,150-5 ,50-1};
        
        auto ret = middle.angle(a,b);
        REQUIRE(ret == Approx(M_PI));
    }
    SECTION("90 deg test should result in M_PI/2")
    {
        distance_t middle = {0,0,0,0};
        distance_t a = {0,1,0,0};
        distance_t b = {1,0,0,0};
        
        auto ret = middle.angle(a,b);
        REQUIRE(ret == Approx(M_PI/2));
    }
    SECTION("90 deg test should result in M_PI/2")
    {
        distance_t middle = {0,0,0,0};
        distance_t a = {0,-1,0,0};
        distance_t b = {1,0,0,0};
        
        auto ret = middle.angle(a,b);
        REQUIRE(ret == Approx(M_PI/2));
    }
}
