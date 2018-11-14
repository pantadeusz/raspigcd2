#include <configuration.hpp>
#include <configuration_json.hpp>
#include <hardware/driver/inmem.hpp>
#include <hardware/stepping.hpp>
#include <movement/steps_generator.hpp>

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
using namespace raspigcd::movement;


TEST_CASE("Movement constant speed", "[movement][steps_generator]")
{
    configuration::global cfg;
    cfg.load_defaults();
    cfg.tick_duration_us = 60;
    std::shared_ptr<motor_layout> motor_layout_ = motor_layout::get_instance(cfg);
    movement::steps_generator const_speed_driver(motor_layout_);
    stepping_sim stepping({0, 0, 0, 0});

    distance_t start_coord = {0, 0, 0, 0};
    steps_t steps = motor_layout_.get()->cartesian_to_steps(start_coord);


    SECTION("Run empty program")
    {
        int n = 0;
        stepping.set_callback([&](const auto&) { n++; });
        stepping.exec({});
        REQUIRE(n == 0);
    }
    SECTION("Generate movement forward and backward ")
    {
        int n = 0;
        using namespace std::chrono_literals;
        auto commands = const_speed_driver.movement_from_to(start_coord, {.v0 = 30, .accel = 0, .max_v = 30}, {1, 3, 2, 0}, cfg.tick_duration());

        stepping.set_callback([&n](const steps_t&) { n++; });
        stepping.current_steps = steps;
        stepping.exec(commands);
        auto steps_result = stepping.current_steps;

        commands = const_speed_driver.movement_from_to({1, 3, 2, 0}, {.v0 = 30, .accel = 0, .max_v = 30}, start_coord, cfg.tick_duration());
        stepping.current_steps = steps_result;
        stepping.exec(commands);
        steps_result = stepping.current_steps;
        REQUIRE(steps_result == steps);
    }

    SECTION("Generate movement forward")
    {
        using namespace std::chrono_literals;
        steps_t steps = {0, 0, 0, 0};
        auto commands = const_speed_driver.movement_from_to({0, 0, 0, 0}, {.v0 = 30, .accel = 0, .max_v = 30}, {-1, 1, 2, 0}, cfg.tick_duration());
        stepping.current_steps = steps;
        stepping.exec(commands);
        steps = stepping.current_steps;
        steps_t expected_steps = motor_layout_.get()->cartesian_to_steps({-1, 1, 2, 0});
        REQUIRE(steps == expected_steps);
    }
    SECTION("Generate movement superfast forward")
    {
        using namespace std::chrono_literals;
        steps_t steps = {0, 0, 0, 0};
        auto commands = const_speed_driver.movement_from_to({0, 0, 0, 0}, {.v0 = 3000, .accel = 0, .max_v = 3000}, {-1, 1, 2, 0}, cfg.tick_duration());
        stepping.current_steps = steps;
        stepping.exec(commands);
        steps = stepping.current_steps;
        steps_t expected_steps = motor_layout_.get()->cartesian_to_steps({-1, 1, 2, 0});
        REQUIRE(steps == expected_steps);
    }

    SECTION("steps_generator::collapse_repeated_steps")
    {
        auto result = steps_generator::collapse_repeated_steps({});
        REQUIRE(result.size() == 0);

        result = steps_generator::collapse_repeated_steps({{
                                                                .b = {{.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}},
                                                                .count = 1}});
        REQUIRE(result.size() == 1);

        result = steps_generator::collapse_repeated_steps(
            {
            {.b = {{.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}}, .count = 1},
            {.b = {{.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}}, .count = 1}
            }
            );
        REQUIRE(result.size() == 1);
        REQUIRE(result[0].count == 2);

        result = steps_generator::collapse_repeated_steps(
            {
            {.b = {{.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}}, .count = 1},
            {.b = {{.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}}, .count = 0}
            }
            );
        REQUIRE(result.size() == 1);
        REQUIRE(result[0].count == 1);

        result = steps_generator::collapse_repeated_steps(
            {
            {.b = {{.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}}, .count = 0},
            {.b = {{.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}}, .count = 0}
            }
            );
        REQUIRE(result.size() == 0);

        result = steps_generator::collapse_repeated_steps(
            {
            {.b = {{.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}}, .count = 1},
            {.b = {{.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}, {.step = 0, .dir = 0}}, .count = 2}
            }
            );
        REQUIRE(result.size() == 1);
        REQUIRE(result[0].count == 3);

        result = steps_generator::collapse_repeated_steps(
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


TEST_CASE("Movement constant speed with timer", "[movement][steps_generator][chrono]")
{
    std::shared_ptr<low_steppers> lsfake = std::make_shared<driver::inmem>();
    configuration::global cfg;
    cfg.load_defaults();
    cfg.tick_duration_us = 60;
    std::shared_ptr<motor_layout> motor_layout_ = motor_layout::get_instance(cfg);
    movement::steps_generator const_speed_driver(motor_layout_);
    stepping_simple_timer stepping(cfg, lsfake);

    distance_t start_coord = {0, 0, 0, 0};
    steps_t steps = motor_layout_.get()->cartesian_to_steps(start_coord);
    distance_t goal_coord = {-1, 1, 2, 0};
    double velocity = 30; // mm/s

    SECTION("Generate movement forward with timer")
    {
        int n = 0;
        using namespace std::chrono_literals;
        auto commands = const_speed_driver.movement_from_to(start_coord, {.v0 = velocity, .accel = 0, .max_v = velocity}, goal_coord, cfg.tick_duration());
        ((driver::inmem*)lsfake.get())->current_steps = steps;
        ((driver::inmem*)lsfake.get())->set_step_callback([&](const auto&) { n++; });

        auto start_time = std::chrono::system_clock::now();
        stepping.exec(commands);
        auto end_time = std::chrono::system_clock::now();
        steps = ((driver::inmem*)lsfake.get())->current_steps;
        steps_t expected_steps = motor_layout_.get()->cartesian_to_steps(goal_coord);
        // 30mm/s;  t = 2.449489742783178/30
        double expected_duration = ((std::sqrt((goal_coord - start_coord).length2()) / velocity));
        double result_duration = (std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000000.0);
        CHECK(result_duration == Approx(expected_duration).epsilon(0.01));
        REQUIRE(steps == expected_steps);
    }

    SECTION("Generate movement superfast forward with timer")
    {
        int n = 0;
        using namespace std::chrono_literals;
        ((driver::inmem*)lsfake.get())->current_steps = steps;
        ((driver::inmem*)lsfake.get())->set_step_callback([&](const auto&) { n++; });
        auto commands = const_speed_driver.movement_from_to({0, 0, 0, 0}, {.v0 = 3000, .accel = 0, .max_v = 3000}, {-1, 1, 2, 0}, cfg.tick_duration());
        auto start_time = std::chrono::system_clock::now();
        stepping.exec(commands);
        auto end_time = std::chrono::system_clock::now();

        double expected_duration = ((std::sqrt((goal_coord - start_coord).length2()) / 3000));
        double result_duration = (std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000000.0);

        steps = ((driver::inmem*)lsfake.get())->current_steps;
        steps_t expected_steps = motor_layout_.get()->cartesian_to_steps({-1, 1, 2, 0});
        REQUIRE(steps == expected_steps);
        CHECK(result_duration > expected_duration);
    }
}

SCENARIO("steps_generator for transitions with accelerations", "[movement][steps_generator][movement_from_to]")
{
    std::shared_ptr<low_steppers> lsfake = std::make_shared<driver::inmem>();
    configuration::global cfg;
    cfg.load_defaults();
    cfg.tick_duration_us = 60;
    std::shared_ptr<motor_layout> motor_layout_ = motor_layout::get_instance(cfg);
    movement::steps_generator steps_generator(motor_layout_);
    stepping_simple_timer stepping(cfg, lsfake);


    steps_t prev_pos = {0, 0, 0, 0};
    std::vector<int> delays_array;
    delays_array.reserve(65535);
    int delay_between_steps = 0;

    ((driver::inmem*)lsfake.get())->set_step_callback([&](const steps_t& pos) {
        if (pos == prev_pos) {
            delay_between_steps++;
        } else {
            delays_array.push_back(delay_between_steps);
            delay_between_steps = 0;
        }
        prev_pos = pos;
    });
    //        worker.exec({cmnd});
    //        auto st = ((driver::inmem*)lsfake.get())->current_steps;


    GIVEN("we have segment to move with no acceleration at all")
    {
        distance_t p0 = {0, 0, 0, 0};
        distance_t p1 = {-3, 1, 2, 0};
        movement::transition_t transition = {
            .v0 = 3,    // initial velocity
            .accel = 0, // acceleration to the next node
            .max_v = 3  // maximal intended velocity that can be performed on this fragment. The most desired speed. The velocity cannot exceed max_v no matter what.
        };


        WHEN("steps are generated")
        {
            auto commands = steps_generator.movement_from_to(p0, transition, p1, cfg.tick_duration());
            steps_t steps = {0, 0, 0, 0};
            ((driver::inmem*)lsfake.get())->current_steps = steps;
            stepping.exec(commands);
            steps = ((driver::inmem*)lsfake.get())->current_steps;
            THEN("the steps should map to expected end position")
            {
                steps_t expected_steps = motor_layout_.get()->cartesian_to_steps({-3, 1, 2, 0});
                REQUIRE(steps == expected_steps);
            }
            THEN("the steps should be with more or less equal delays")
            {
                int default_delay = std::accumulate(delays_array.begin(), delays_array.end(), 0.0) / delays_array.size();
                for (unsigned i = 1; i < delays_array.size(); i++) {
                    INFO(i);
                    REQUIRE(delays_array[i] >= default_delay - 1);
                    REQUIRE(delays_array[i] <= default_delay + 1);
                }
            }
        }
    }


    GIVEN("we have segment to move with acceleration")
    {
        distance_t p0 = {0, 0, 0, 0};
        distance_t p1 = {-3, 1, 2, 0};
        p1 = p1 * 0.1;
        movement::transition_t transition = {
            .v0 = 3,      // initial velocity
            .accel = 100, // acceleration to the next node
            .max_v = 16   // maximal intended velocity that can be performed on this fragment. The most desired speed. The velocity cannot exceed max_v no matter what.
        };


        WHEN("speed at then end of movement is calculated")
        {
            double v1 = steps_generator.velocity_after_from_to(p0, transition, p1, cfg.tick_duration());
            THEN("The speed should be higher than initial speed")
            {
                REQUIRE(v1 > transition.v0);
            }
        }

        WHEN("steps are generated")
        {
            auto commands = steps_generator.movement_from_to(p0, transition, p1, cfg.tick_duration());
            steps_t steps = {0, 0, 0, 0};
            ((driver::inmem*)lsfake.get())->current_steps = steps;
            stepping.exec(commands);
            steps = ((driver::inmem*)lsfake.get())->current_steps;
            THEN("the steps should map to expected end position")
            {
                distance_t p1_expected = {-3, 1, 2, 0};
                p1_expected = p1_expected * 0.1;
                steps_t expected_steps = motor_layout_.get()->cartesian_to_steps(p1_expected);
                REQUIRE(steps == expected_steps);
            }
            THEN("the speed should increase over time")
            {
                INFO(delays_array[0]);
                INFO(delays_array.back());
                REQUIRE((delays_array[0] - delays_array.back()) > 6);
            }
        }
        WHEN("the speed exceeds max_v, the exception should be thrown")
        {
            REQUIRE_THROWS_WITH(steps_generator.movement_from_to(p0, transition, p1 * 10.0, cfg.tick_duration()), "velocity exceeds max_v");
        }
    }
}


SCENARIO("steps_generator to calculate velocity at the end of movement", "[movement][steps_generator][velocity_after_from_to]")
{
    std::shared_ptr<low_steppers> lsfake = std::make_shared<driver::inmem>();
    configuration::global cfg;
    cfg.load_defaults();
    cfg.tick_duration_us = 60;
    std::shared_ptr<motor_layout> motor_layout_ = motor_layout::get_instance(cfg);
    movement::steps_generator steps_generator(motor_layout_);
    stepping_simple_timer stepping(cfg, lsfake);

    GIVEN("we have segment to move with acceleration")
    {
        distance_t p0 = {0, 0, 0, 0};
        distance_t p1 = {0, 0, 0, 0};
        movement::transition_t transition = {
            .v0 = 1,     // initial velocity
            .accel = 10, // acceleration to the next node
            .max_v = 10  // maximal intended velocity that can be performed on this fragment. The most desired speed. The velocity cannot exceed max_v no matter what.
        };
        // v = a*t  + v0
        // p = 0 + v0*t + 0.5*a*t*t
        double T = 0.5; // 0.5 second
        p1[0] = transition.v0 * T + 0.5 * transition.accel * T * T;
        WHEN("the velocity at the end of movement is calculated")
        {
            double v1 = steps_generator.velocity_after_from_to(p0, transition, p1, cfg.tick_duration());
            THEN("it should be almost equal to true value")
            {
                double v_expected = transition.accel * T + transition.v0;
                INFO(v1);
                INFO(v_expected);
                REQUIRE(v1 == Approx(v_expected).epsilon(0.001));
                REQUIRE(v1 <= v_expected);
            }
        }
    }

    GIVEN("we have segment to move with no acceleration")
    {
        distance_t p0 = {0, 0, 0, 0};
        distance_t p1 = {0, 0, 0, 0};
        movement::transition_t transition = {
            .v0 = 1,    // initial velocity
            .accel = 0, // acceleration to the next node
            .max_v = 10 // maximal intended velocity that can be performed on this fragment. The most desired speed. The velocity cannot exceed max_v no matter what.
        };
        // v = a*t  + v0
        // p = 0 + v0*t + 0.5*a*t*t
        double T = 0.5; // 0.5 second
        p1[0] = transition.v0 * T + 0.5 * transition.accel * T * T;
        WHEN("the velocity at the end of movement is calculated")
        {
            double v1 = steps_generator.velocity_after_from_to(p0, transition, p1, cfg.tick_duration());
            THEN("it should be almost equal to true value")
            {
                double v_expected = transition.accel * T + transition.v0;
                INFO(v1);
                INFO(v_expected);
                REQUIRE(v1 == Approx(v_expected).epsilon(0.001));
                REQUIRE(v1 <= v_expected);
            }
        }
    }
}
