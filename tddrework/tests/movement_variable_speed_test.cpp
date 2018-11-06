#include <configuration.hpp>
#include <configuration_json.hpp>
#include <hardware/stepping.hpp>
#include <movement/variable_speed.hpp>

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

SCENARIO( "variable speed and accelerations", "[movement][variable_speed]" ) {
    std::shared_ptr<low_steppers> lsfake(new low_steppers_fake());
    configuration::global cfg;
    cfg.load_defaults();
    cfg.tick_duration_us = 60;
    cfg.max_no_accel_velocity_mm_s = {5, 5, 5, 5};
    double max_speed_no_accel = cfg.max_no_accel_velocity_mm_s[0];
    double acceleration = 100;
    std::shared_ptr<motor_layout> motor_layout_ = motor_layout::get_instance(cfg);
    movement::variable_speed variable_speed_driver(motor_layout_, max_speed_no_accel, acceleration, 150, cfg.tick_duration());
    stepping_simple_timer stepping(cfg, lsfake);

    distance_t start_coord = {0, 0, 0, 0};
    steps_t steps = motor_layout_.get()->cartesian_to_steps(start_coord);
    distance_t goal_coord = {-1, 1, 2, 0};
    double velocity = 30; // mm/s

    GIVEN("there is an empty program to run") {
        std::list<std::variant<distance_t,double> > empty_program;
        WHEN("the program is converted to actions") {
            std::list<std::variant<distance_t,transition_t> > 
                result = variable_speed_driver.movement_intet_to_point_speeds_v( empty_program);
            THEN("actions should be empty") {
                REQUIRE(result.size() == 0);
            }
        }
    }

    GIVEN("there is a program with constant and safe speed") {
        std::list<std::variant<distance_t,double> > simple_program;
        simple_program.push_back(distance_t{0,0,0,0});
        simple_program.push_back(2);
        simple_program.push_back(distance_t{1,0,0,0});
        simple_program.push_back(2);
        simple_program.push_back(distance_t{1,1,0,0});
        simple_program.push_back(2);
        simple_program.push_back(distance_t{0,1,0,0});
        simple_program.push_back(2);
        simple_program.push_back(distance_t{0,0,0,0});
        
        WHEN("the program is converted to actions") {
            std::list<std::variant<distance_t,transition_t> > 
                result = variable_speed_driver.movement_intet_to_point_speeds_v( simple_program );
            THEN("the result should be of the same size as simple program") {
                REQUIRE(result.size() == simple_program.size());
            }
        }
    }
}
