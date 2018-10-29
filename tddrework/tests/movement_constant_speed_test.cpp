#include <configuration.hpp>
#include <configuration_json.hpp>
#include <hardware_stepping.hpp>
#include <movement_constant_speed.hpp>

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

class low_steppers_fake : public hardware::low_steppers
{
public:
    int counters[RASPIGCD_HARDWARE_DOF];
    std::vector<bool> enabled;
    void do_step(const single_step_command* b)
    {
        for (int i = 0; i < RASPIGCD_HARDWARE_DOF; i++) {
            if (b[i].step == 1) {
                counters[i] += b[i].dir * 2 - 1;
            }
        }
    };
    void enable_steppers(const std::vector<bool> en)
    {
        enabled = en;
    };
    low_steppers_fake()
    {
        for (int i = 0; i < RASPIGCD_HARDWARE_DOF; i++) {
            counters[i] = 0;
        }
        enabled = std::vector<bool>(false, RASPIGCD_HARDWARE_DOF);
    }
};


TEST_CASE("Movement constant speed", "[movement][constant_speed]")
{
    std::shared_ptr<low_steppers> lsfake(new low_steppers_fake());
    configuration::global cfg;
    cfg.load_defaults();
    cfg.tick_duration_us = 60;
    std::shared_ptr<motor_layout> motor_layout_ = motor_layout::get_instance(cfg);
    movement::constant_speed const_speed_driver(motor_layout_);
    stepping_simple_timer stepping(cfg, lsfake);

    distance_t start_coord = {0, 0, 0, 0};
    steps_t steps = motor_layout_.get()->cartesian_to_steps(start_coord);
    distance_t goal_coord = {-1, 1, 2, 0};
    double velocity = 30; // mm/s


    SECTION("Run empty program")
    {
        int n = 0;
        stepping.exec({0, 0, 0, 0}, {}, [&](const auto&) { n++; });
        REQUIRE(n == 0);
    }
    SECTION("Generate movement forward and backward ")
    {
        int n = 0;
        using namespace std::chrono_literals;
        auto commands = const_speed_driver.goto_xyz(start_coord, {1, 3, 2, 0}, 30, cfg.tick_duration());
        auto steps_result = stepping.exec(steps,
            commands,
            [&n](const steps_t&) {n++;});
        commands = const_speed_driver.goto_xyz({1, 3, 2, 0}, start_coord, 30, cfg.tick_duration());
        steps_result = stepping.exec(steps_result,
            commands,
            [&n](const steps_t&) {n++;});
        REQUIRE (steps_result == steps);
    }

    SECTION("Generate movement forward")
    {
        int n = 0;
        using namespace std::chrono_literals;
        steps_t steps = {0, 0, 0, 0};
        auto commands = const_speed_driver.goto_xyz({0, 0, 0, 0}, {-1, 1, 2, 0}, 30, cfg.tick_duration());
        steps = stepping.exec(steps,
            commands,
            [&n](const steps_t&) {n++;});
        steps_t expected_steps = motor_layout_.get()->cartesian_to_steps({-1, 1, 2, 0});
        REQUIRE (steps == expected_steps);
    }
    SECTION("Generate movement superfast forward")
    {
        int n = 0;
        using namespace std::chrono_literals;
        steps_t steps = {0, 0, 0, 0};
        auto commands = const_speed_driver.goto_xyz({0, 0, 0, 0}, {-1, 1, 2, 0}, 3000, cfg.tick_duration());
        steps = stepping.exec(steps,
            commands,
            [&n](const steps_t&) {n++;});
        steps_t expected_steps = motor_layout_.get()->cartesian_to_steps({-1, 1, 2, 0});
        REQUIRE (steps == expected_steps);
    }
    SECTION("Generate movement forward with timer")
    {
        int n = 0;
        using namespace std::chrono_literals;
        auto commands = const_speed_driver.goto_xyz(start_coord, goal_coord, velocity, cfg.tick_duration());
        auto start_time = std::chrono::system_clock::now();
        steps = stepping.exec(steps,
            commands,
            [&n](const steps_t&) {n++;});
        auto end_time = std::chrono::system_clock::now();
        steps_t expected_steps = motor_layout_.get()->cartesian_to_steps(goal_coord);
        // 30mm/s;  t = 2.449489742783178/30
        double expected_duration = ((std::sqrt((goal_coord - start_coord).length2())/velocity));
        double result_duration = (std::chrono::duration_cast<std::chrono::microseconds> (end_time - start_time).count()/1000000.0);
        CHECK (result_duration == Approx(expected_duration).epsilon(0.01) );
        REQUIRE (steps == expected_steps);
    }
    SECTION("Generate movement superfast forward with timer")
    {
        int n = 0;
        using namespace std::chrono_literals;
        steps_t steps = {0, 0, 0, 0};
        auto commands = const_speed_driver.goto_xyz({0, 0, 0, 0}, {-1, 1, 2, 0}, 3000, cfg.tick_duration());
        steps = stepping.exec(steps,
            commands,
            [&n](const steps_t&) {n++;});
        steps_t expected_steps = motor_layout_.get()->cartesian_to_steps({-1, 1, 2, 0});
        REQUIRE (steps == expected_steps);
    }
}
