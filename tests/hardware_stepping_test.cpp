#include <configuration.hpp>
#include <configuration_json.hpp>
#include <hardware/stepping.hpp>
#include <hardware/driver/inmem.hpp>
#include <hardware/driver/low_timers_fake.hpp>

#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>

#include <chrono>
#include <thread>
#include <vector>

using namespace raspigcd;
using namespace raspigcd::configuration;
using namespace raspigcd::hardware;


TEST_CASE("Hardware stepping_sim", "[hardware_stepping][stepping_sim]")
{
    stepping_sim worker({0,0,0,0});

    SECTION("Run empty program")
    {
        int n = 0;
        worker.set_callback([&](const auto&) { n++; });
        worker.exec({});
        REQUIRE(n == 0);
    }

    SECTION("Run one command program")
    {
        int n = 0;
        worker.set_callback([&](const auto&) { n++; });
        worker.exec({{{{.step=0, .dir=0}, {.step=0, .dir=0}, {.step=0, .dir=0}, {.step=0, .dir=0}},1}});
        REQUIRE(n == 1);
    }
    SECTION("Run one step in each positive  direction")
    {
        for (int i = 0; i < 4; i++) {
            int n = 0;
            multistep_command cmnd;
            cmnd.count = 1;
            cmnd.b[0].step = 0;
            cmnd.b[0].dir = 0;
            cmnd.b[1].step = 0;
            cmnd.b[1].dir = 0;
            cmnd.b[2].step = 0;
            cmnd.b[2].dir = 0;
            cmnd.b[3].step = 0;
            cmnd.b[3].dir = 0;
            cmnd.b[i].step = 1;
            cmnd.b[i].dir = 1;
            worker.set_callback([&](const auto&) { n++; });
            worker.current_steps = {5, 6, 7, 8};
            worker.exec({cmnd});
            REQUIRE(n == 1);
            steps_t cmpto = {5, 6, 7, 8};
            cmpto[i] += 1;
            REQUIRE(worker.current_steps == cmpto);
        }
    }
    SECTION("Run one step in each negative direction")
    {
        for (int i = 0; i < 4; i++) {
            int n = 0;
            multistep_command cmnd;
            cmnd.count = 1;
            cmnd.b[0].step = 0;
            cmnd.b[0].dir = 0;
            cmnd.b[1].step = 0;
            cmnd.b[1].dir = 0;
            cmnd.b[2].step = 0;
            cmnd.b[2].dir = 0;
            cmnd.b[3].step = 0;
            cmnd.b[3].dir = 0;
            cmnd.b[i].step = 1;
            cmnd.b[i].dir = 0;
            worker.set_callback([&](const auto&) { n++; });
            worker.current_steps = {1, 2, 3, 4};
            worker.exec({cmnd});
            REQUIRE(n == 1);
            steps_t cmpto = {1, 2, 3, 4};
            cmpto[i] -= 1;
            REQUIRE(worker.current_steps == cmpto);
        }
    }
}


TEST_CASE("Hardware stepping_simple_timer", "[hardware_stepping][stepping_simple_timer]")
{
    std::shared_ptr<low_steppers> lsfake(new driver::inmem());
    std::shared_ptr<low_timers> ltfake = std::make_shared<driver::low_timers_fake>();
    stepping_simple_timer worker(60, lsfake, ltfake);

    SECTION("Run empty program")
    {
        int n = 0;
        ((driver::inmem*)lsfake.get())->current_steps = {0, 0, 0, 0};
        ((driver::inmem*)lsfake.get())->set_step_callback([&](const auto&) { n++; });
        worker.exec({});
        REQUIRE(n == 0);
    }

    SECTION("Run one command program")
    {
        int n = 0;
        ((driver::inmem*)lsfake.get())->current_steps = {0, 0, 0, 0};
        ((driver::inmem*)lsfake.get())->set_step_callback([&](const auto&) { n++; });
        worker.exec({{.b = {{0, 0}, {0, 0}, {0, 0}, {0, 0}}, .count = 1}});
        REQUIRE(n == 1);
    }
    SECTION("Run one step in each positive  direction")
    {
        for (int i = 0; i < 4; i++) {
            int n = 0;
            multistep_command cmnd;
            cmnd.count = 1;
            cmnd.b[0].step = 0;
            cmnd.b[0].dir = 0;
            cmnd.b[1].step = 0;
            cmnd.b[1].dir = 0;
            cmnd.b[2].step = 0;
            cmnd.b[2].dir = 0;
            cmnd.b[3].step = 0;
            cmnd.b[3].dir = 0;
            cmnd.b[i].step = 1;
            cmnd.b[i].dir = 1;
            ((driver::inmem*)lsfake.get())->current_steps = {5, 6, 7, 8};
            ((driver::inmem*)lsfake.get())->set_step_callback([&](const auto&) { n++; });
            worker.exec({cmnd});
            REQUIRE(n == 1);
            steps_t cmpto = {5, 6, 7, 8};
            cmpto[i] += 1;
            REQUIRE(((driver::inmem*)lsfake.get())->current_steps == cmpto);
            REQUIRE(((driver::inmem*)lsfake.get())->counters[i] == 1);
        }
    }
    SECTION("Run one step in each negative direction")
    {
        for (int i = 0; i < 4; i++) {
            int n = 0;
            multistep_command cmnd;
            cmnd.count = 1;
            cmnd.b[0].step = 0;
            cmnd.b[0].dir = 0;
            cmnd.b[1].step = 0;
            cmnd.b[1].dir = 0;
            cmnd.b[2].step = 0;
            cmnd.b[2].dir = 0;
            cmnd.b[3].step = 0;
            cmnd.b[3].dir = 0;
            cmnd.b[i].step = 1;
            cmnd.b[i].dir = 0;
            ((driver::inmem*)lsfake.get())->current_steps = {1, 2, 3, 4};
            ((driver::inmem*)lsfake.get())->set_step_callback([&](const auto&) { n++; });
            worker.exec({cmnd});
            REQUIRE(n == 1);
            steps_t cmpto = {1, 2, 3, 4};
            cmpto[i] -= 1;
            REQUIRE(((driver::inmem*)lsfake.get())->current_steps == cmpto);
            REQUIRE(((driver::inmem*)lsfake.get())->counters[i] == -1);
        }
    }

    SECTION("stepping break counter should be correct 1") {
        multistep_commands_t commands_to_do;

        for (int i = 0; i < 4; i++) {
            multistep_command cmnd;
            cmnd.count = 1+i;
            cmnd.b[0].step = 0;
            cmnd.b[0].dir = 0;
            cmnd.b[1].step = 0;
            cmnd.b[1].dir = 0;
            cmnd.b[2].step = 0;
            cmnd.b[2].dir = 0;
            cmnd.b[3].step = 0;
            cmnd.b[3].dir = 0;
            cmnd.b[i].step = 1;
            cmnd.b[i].dir = 0;
            commands_to_do.push_back(cmnd);
        }

        std::list<steps_t> steps_list = hardware_commands_to_steps(commands_to_do);
        steps_t last_step = hardware_commands_to_last_position_after_given_steps(commands_to_do, 1 );
        REQUIRE(last_step == std::vector<steps_t>(steps_list.begin(),steps_list.end()).at(0));
    }

    SECTION("stepping break counter should be correct - 2") {
        multistep_commands_t commands_to_do;

        for (int i = 0; i < 4; i++) {
            multistep_command cmnd;
            cmnd.count = 1+i;
            cmnd.b[0].step = 0;
            cmnd.b[0].dir = 0;
            cmnd.b[1].step = 0;
            cmnd.b[1].dir = 0;
            cmnd.b[2].step = 0;
            cmnd.b[2].dir = 0;
            cmnd.b[3].step = 0;
            cmnd.b[3].dir = 0;
            cmnd.b[i].step = 1;
            cmnd.b[i].dir = 0;
            commands_to_do.push_back(cmnd);
        }

        std::list<steps_t> steps_list = hardware_commands_to_steps(commands_to_do);
        steps_t last_step = hardware_commands_to_last_position_after_given_steps(commands_to_do, 5 );
        REQUIRE(last_step == std::vector<steps_t>(steps_list.begin(),steps_list.end()).at(4));
    }

    SECTION("stepping break counter should be correct - last") {
        multistep_commands_t commands_to_do;

        for (int i = 0; i < 4; i++) {
            multistep_command cmnd;
            cmnd.count = 1+i;
            cmnd.b[0].step = 0;
            cmnd.b[0].dir = 0;
            cmnd.b[1].step = 0;
            cmnd.b[1].dir = 0;
            cmnd.b[2].step = 0;
            cmnd.b[2].dir = 0;
            cmnd.b[3].step = 0;
            cmnd.b[3].dir = 0;
            cmnd.b[i].step = 1;
            cmnd.b[i].dir = 0;
            commands_to_do.push_back(cmnd);
        }

        std::list<steps_t> steps_list = hardware_commands_to_steps(commands_to_do);
        steps_t last_step = hardware_commands_to_last_position_after_given_steps(commands_to_do, -1 );
        REQUIRE(last_step == steps_list.back());
    }
}
