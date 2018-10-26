#include <hardware_stepping.hpp>
#include <configuration.hpp>
#include <configuration_json.hpp>

#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>

#include <vector>
#include <chrono>
#include <thread>

using namespace raspigcd;
using namespace raspigcd::configuration;
using namespace raspigcd::hardware;



TEST_CASE( "Hardware stepping_sim", "[hardware_stepping][stepping_sim]" ) {
    stepping_sim worker;

    SECTION( "Run empty program" ) {
        int n = 0;
        worker.exec({0,0,0,0},{},[&](const auto &){n++;});
        REQUIRE(n == 0);
    }

    SECTION( "Run one command program" ) {
        int n = 0;
        worker.exec({0,0,0,0},{ {cmnd:{b:{{0,0},{0,0},{0,0},{0,0}},repeat:1}} },[&](const auto &){n++;});
        REQUIRE(n == 1);
    }
    SECTION( "Run one step in each positive  direction" ) {
        for (int i = 0; i < 4; i++) {
            int n = 0;
            multistep_command cmnd;
            cmnd.cmnd.repeat = 1;
            cmnd.cmnd.b[0].step = 0;
            cmnd.cmnd.b[0].dir = 0;
            cmnd.cmnd.b[1].step = 0;
            cmnd.cmnd.b[1].dir = 0;
            cmnd.cmnd.b[2].step = 0;
            cmnd.cmnd.b[2].dir = 0;
            cmnd.cmnd.b[3].step = 0;
            cmnd.cmnd.b[3].dir = 0;
            cmnd.cmnd.b[i].step = 1;
            cmnd.cmnd.b[i].dir = 1;
            auto st = worker.exec({5,6,7,8},{cmnd},[&](const auto &){n++;});
            REQUIRE(n == 1);
            steps_t cmpto = {5,6,7,8};
            cmpto[i] += 1;
            REQUIRE(st == cmpto);
        }
    }
    SECTION( "Run one step in each negative direction" ) {
        for (int i = 0; i < 4; i++) {
            int n = 0;
            multistep_command cmnd;
            cmnd.cmnd.repeat = 1;
            cmnd.cmnd.b[0].step = 0;
            cmnd.cmnd.b[0].dir = 0;
            cmnd.cmnd.b[1].step = 0;
            cmnd.cmnd.b[1].dir = 0;
            cmnd.cmnd.b[2].step = 0;
            cmnd.cmnd.b[2].dir = 0;
            cmnd.cmnd.b[3].step = 0;
            cmnd.cmnd.b[3].dir = 0;
            cmnd.cmnd.b[i].step = 1;
            cmnd.cmnd.b[i].dir = 0;
            auto st = worker.exec({1,2,3,4},{cmnd},[&](const auto &){n++;});
            REQUIRE(n == 1);
            steps_t cmpto = {1,2,3,4};
            cmpto[i] -= 1;
            REQUIRE(st == cmpto);
        }
    }

}
