#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>
#include <chrono>
#include <gcd/path_intent_executor.hpp>
#include <hardware/driver/inmem.hpp>
#include <hardware/driver/low_buttons_fake.hpp>
#include <hardware/driver/low_spindles_pwm_fake.hpp>
#include <hardware/driver/low_timers_fake.hpp>
#include <hardware/stepping.hpp>
#include <thread>
#include <vector>

using namespace raspigcd;
using namespace raspigcd::movement;
using namespace raspigcd::configuration;


TEST_CASE("path_intent_executor test whole program and motors", "[gcd][path_intent_executor][e2e]")
{

    /// this is the complete initialization for testing.
    configuration::global actuators_cfg{};
    actuators_cfg.motion_layout = COREXY; //"corexy";
    actuators_cfg.scale = {1.0, 1.0, 1.0, 1.0};
    actuators_cfg.steppers = {
        stepper(27, 10, 22, 100.0),
        stepper(4, 10, 17, 100.0),
        stepper(9, 10, 11, 100.0),
        stepper(0, 10, 5, 100.0)};
    actuators_cfg.tick_duration_us = 50;

    actuators_cfg.max_accelerations_mm_s2 = {200.0, 200.0, 200.0, 200.0};
    actuators_cfg.max_velocity_mm_s = {220.0, 220.0, 110.0, 220.0};  ///<maximal velocity on axis in mm/s
    actuators_cfg.max_no_accel_velocity_mm_s = {50.0, 50.0, 50.0, 50.0}; ///<maximal velocity on axis in mm/s


    raspigcd::gcd::path_intent_executor executor;
    
    raspigcd::gcd::gcode_interpreter_objects_t objs{};
    objs.buttons = std::make_shared<hardware::driver::low_buttons_fake>();
    objs.steppers = std::make_shared<hardware::driver::inmem>();
    objs.spindles_pwm = std::make_shared<hardware::driver::low_spindles_pwm_fake>();
    objs.timers = std::make_shared<hardware::driver::low_timers_fake>();
    objs.motor_layout = hardware::motor_layout::get_instance(actuators_cfg);
    objs.configuration.motion_layout = actuators_cfg.motion_layout;
    objs.configuration.scale = actuators_cfg.scale;
    objs.configuration.steppers = actuators_cfg.steppers;
    objs.configuration = actuators_cfg;
    
    objs.stepping = std::make_shared<hardware::stepping_simple_timer>(objs.configuration, objs.steppers, objs.timers);

    hardware::driver::inmem* inmem_ptr = (hardware::driver::inmem*)objs.steppers.get();
    hardware::driver::low_timers_fake* low_timers_fake_ptr = (hardware::driver::low_timers_fake*)objs.timers.get();
    hardware::driver::low_spindles_pwm_fake* low_spindles_pwm_fake_ptr = (hardware::driver::low_spindles_pwm_fake*)objs.spindles_pwm.get();

  /// ok, back to work

    SECTION("program movement execution")
    {
        executor.set_gcode_interpreter_objects(objs);
        std::list<int> rec;

        inmem_ptr->on_enable_steppers = [&rec](auto) { rec.push_back(0); };
        low_timers_fake_ptr->on_wait_s = [&rec](auto) { rec.push_back(1); };
        low_spindles_pwm_fake_ptr->on_spindle_pwm_power = [&rec](auto, auto) { rec.push_back(2); };

        auto result = executor.execute(
            {
                distance_t{0,0,0,0},
                movement::path_intentions::move_t(48.0),
                distance_t{1,2,3,4},
                movement::path_intentions::move_t(20.0),
                distance_t{2,1,0,2}                
            });
        REQUIRE(inmem_ptr->current_steps == objs.motor_layout.get()->cartesian_to_steps(distance_t{2,1,0,2}));
        //REQUIRE(rec == std::list<int>{2, 1, 1, 2, 1});
    }
    SECTION("program movement execution with coordinates reset")
    {
        executor.set_gcode_interpreter_objects(objs);
        std::list<int> rec;

        inmem_ptr->on_enable_steppers = [&rec](auto) { rec.push_back(0); };
        low_timers_fake_ptr->on_wait_s = [&rec](auto) { rec.push_back(1); };
        low_spindles_pwm_fake_ptr->on_spindle_pwm_power = [&rec](auto, auto) { rec.push_back(2); };

        auto result = executor.execute(
            {
                distance_t{0,0,0,0},
                movement::path_intentions::move_t(48.0),
                distance_t{1,2,3,4},
                movement::path_intentions::pause_t{.delay_s=0.0},
                distance_t{0,0,0,0},
                movement::path_intentions::move_t(20.0),
                distance_t{2,1,0,2}    ,            
            });
        REQUIRE(inmem_ptr->current_steps == objs.motor_layout.get()->cartesian_to_steps(distance_t{3,3,3,6}));
        REQUIRE(rec == std::list<int>{1});
    }

    SECTION("program movement execution with coordinates reset and spindle and motors")
    {
        executor.set_gcode_interpreter_objects(objs);
        std::list<int> rec;

        inmem_ptr->on_enable_steppers = [&rec](auto) { rec.push_back(0); };
        low_timers_fake_ptr->on_wait_s = [&rec](auto) { rec.push_back(1); };
        low_spindles_pwm_fake_ptr->on_spindle_pwm_power = [&rec](auto, auto) { rec.push_back(2); };

        auto result = executor.execute(
            {
                movement::path_intentions::spindle_t{.delay_s = 0.01, .spindle = {{0, 1.0}}},
                movement::path_intentions::motor_t{.delay_s = 0.001, .motor = {true,true,true,true}},
                distance_t{0,0,0,0},
                movement::path_intentions::move_t(48.0),
                distance_t{1,2,3,4},
                movement::path_intentions::pause_t{.delay_s=0.0},
                distance_t{0,0,0,0},
                movement::path_intentions::move_t(20.0),
                distance_t{2,1,0,2},
                movement::path_intentions::spindle_t{.delay_s = 0.01, .spindle = {{0, 0.0}}},
                movement::path_intentions::motor_t{.delay_s = 0.001, .motor = {false,false,false,false}},
            });
        REQUIRE(inmem_ptr->current_steps == objs.motor_layout.get()->cartesian_to_steps(distance_t{3,3,3,6}));
        REQUIRE(rec == std::list<int>{2,1,0,1,1,2,1,0,1});
    }
}
