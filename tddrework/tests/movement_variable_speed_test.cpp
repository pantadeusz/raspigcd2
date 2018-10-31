#include <configuration.hpp>
#include <configuration_json.hpp>
#include <hardware_stepping.hpp>
#include <movement_variable_speed.hpp>

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


TEST_CASE("Movement variable speed", "[movement][variable_speed]")
{
    std::shared_ptr<low_steppers> lsfake(new low_steppers_fake());
    configuration::global cfg;
    cfg.load_defaults();
    cfg.tick_duration_us = 60;
    std::shared_ptr<motor_layout> motor_layout_ = motor_layout::get_instance(cfg);
    movement::variable_speed variable_speed_driver(motor_layout_, 5, 100, 150, cfg.tick_duration());
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

    SECTION("Intentions to set of acceleration sequences for empty")
    {
        std::list<var_speed_intentions_t> intended_moves;
        auto move_segments = variable_speed_driver.movement_intet_to_point_speeds(intended_moves);
        REQUIRE(move_segments.size() == 0);
    }
    SECTION("Intention with one value that does not need to accelerate")
    {
        std::list<var_speed_intentions_t> intended_moves;
        intended_moves.push_back({.p0 = {0, 0, 0, 0},
            .p1 = {2, 3, 4, 0},
            .intended_speed = 3});
        auto move_segments = variable_speed_driver.movement_intet_to_point_speeds(intended_moves);
        REQUIRE(move_segments.size() == 2);
        REQUIRE(move_segments.front().second == move_segments.back().second);
        REQUIRE(move_segments.front().first == intended_moves.back().p0);
        REQUIRE(move_segments.back().first == intended_moves.back().p1);
    }
    SECTION("Intention with multiple values that does not need to accelerate")
    {
        std::list<var_speed_intentions_t> intended_moves;
        intended_moves.push_back({.p0 = {0, 0, 0, 0},
            .p1 = {2, 3, 4, 0},
            .intended_speed = 3});
        intended_moves.push_back({.p0 = {2, 3, 4, 0},
            .p1 = {1, 2, 3, 0},
            .intended_speed = 2});
        intended_moves.push_back({.p0 = {1, 2, 3, 0},
            .p1 = {2, 3, 4, 0},
            .intended_speed = 1});
        auto move_segments = variable_speed_driver.movement_intet_to_point_speeds(intended_moves);

        std::vector<var_speed_intentions_t> im_vect (intended_moves.begin(),intended_moves.end());
        std::vector<var_speed_pointspeed_t> result_vect (move_segments.begin(),move_segments.end());
        REQUIRE(result_vect.size() == 6);
        REQUIRE(result_vect[0].second == result_vect[1].second);
        REQUIRE(result_vect[1].second != result_vect[2].second);
        REQUIRE(result_vect[2].second == result_vect[3].second);
        REQUIRE(result_vect[3].second != result_vect[4].second);
        REQUIRE(result_vect[4].second == result_vect[5].second);

        REQUIRE(result_vect[0].first == im_vect[0].p0);
        REQUIRE(result_vect[1].first == im_vect[0].p1);
        REQUIRE(result_vect[2].first == im_vect[1].p0);
        REQUIRE(result_vect[3].first == im_vect[1].p1);
        REQUIRE(result_vect[4].first == im_vect[2].p0);
        REQUIRE(result_vect[5].first == im_vect[2].p1);

    }
    /*   SECTION("Generate movement forward and backward ")
    {
        int n = 0;
        using namespace std::chrono_literals;
        auto commands = variable_speed_driver.goto_xyz(start_coord, {1, 3, 2, 0}, 30, cfg.tick_duration());
        auto steps_result = stepping.exec(steps,
            commands,
            [&n](const steps_t&) {n++;});
        commands = variable_speed_driver.goto_xyz({1, 3, 2, 0}, start_coord, 30, cfg.tick_duration());
        steps_result = stepping.exec(steps_result,
            commands,
            [&n](const steps_t&) {n++;});
        REQUIRE (steps_result == steps);
    }*/
    /*
    SECTION("Generate movement forward with timer")
    {
        int n = 0;
        using namespace std::chrono_literals;
        auto commands = variable_speed_driver.goto_xyz(start_coord, goal_coord, velocity, cfg.tick_duration());
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
        auto commands = variable_speed_driver.goto_xyz({0, 0, 0, 0}, {-1, 1, 2, 0}, 3000, cfg.tick_duration());
        steps = stepping.exec(steps,
            commands,
            [&n](const steps_t&) {n++;});
        steps_t expected_steps = motor_layout_.get()->cartesian_to_steps({-1, 1, 2, 0});
        REQUIRE (steps == expected_steps);
    }*/
}
