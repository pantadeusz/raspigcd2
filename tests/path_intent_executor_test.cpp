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

// void path_intent_executor::execute_pure_path_intent(const movement::path_intent_t& path_intent) {
TEST_CASE("path_intent_executor execute_pure_path_intent constructor tests", "[gcd][path_intent_executor][execute_pure_path_intent]")
{
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
    
    objs.stepping = std::make_shared<hardware::stepping_simple_timer>(objs.configuration, objs.steppers);



    hardware::driver::inmem* inmem_ptr = (hardware::driver::inmem*)objs.steppers.get();
    hardware::driver::low_timers_fake* low_timers_fake_ptr = (hardware::driver::low_timers_fake*)objs.timers.get();
    hardware::driver::low_spindles_pwm_fake* low_spindles_pwm_fake_ptr = (hardware::driver::low_spindles_pwm_fake*)objs.spindles_pwm.get();

    SECTION("try to execute something else then allowed")
    {
        executor.set_gcode_interpreter_objects(objs);
        REQUIRE_THROWS(executor.execute_pure_path_intent({movement::path_intentions::motor_t{.delay_s = 0.01, .motor = {false, false, false, false}}}));
        REQUIRE_THROWS(executor.execute_pure_path_intent({movement::path_intentions::pause_t{.delay_s = 0.1}}));
        REQUIRE_THROWS(executor.execute_pure_path_intent({movement::path_intentions::spindle_t{.delay_s = 0.01, .spindle = {{0, 1.0}}}}));
    }

    SECTION("execute simple program and check result")
    {
        executor.set_gcode_interpreter_objects(objs);

        executor.execute_pure_path_intent({
            distance_t{0,0,0,0},
            movement::path_intentions::move_t(60.0),
            distance_t{1,2,3,4},
        });
        REQUIRE(inmem_ptr->current_steps == objs.motor_layout.get()->cartesian_to_steps(distance_t{1,2,3,4}));
    }

    SECTION("execute simple program and check result")
    {
        executor.set_gcode_interpreter_objects(objs);
        executor.execute_pure_path_intent({
            distance_t{0,0,0,0},
            movement::path_intentions::move_t(48.0),
            distance_t{1,2,3,4},
            movement::path_intentions::move_t(20.0),
            distance_t{2,1,0,2},
        });
        REQUIRE(inmem_ptr->current_steps == objs.motor_layout.get()->cartesian_to_steps(distance_t{2,1,0,2}));
    }
}

TEST_CASE("path_intent_executor constructor tests basic and motors", "[gcd][path_intent_executor][motors]")
{
    configuration::actuators_organization actuators_cfg;
    actuators_cfg.motion_layout = COREXY; //"corexy";
    actuators_cfg.scale = {1.0, 1.0, 1.0, 1.0};
    actuators_cfg.steppers = {
        stepper(27, 10, 22, 100.0),
        stepper(4, 10, 17, 100.0),
        stepper(9, 10, 11, 100.0),
        stepper(0, 10, 5, 100.0)};

    raspigcd::gcd::path_intent_executor executor;
    raspigcd::gcd::gcode_interpreter_objects_t objs;
    objs.buttons = std::make_shared<hardware::driver::low_buttons_fake>();
    objs.steppers = std::make_shared<hardware::driver::inmem>();
    objs.spindles_pwm = std::make_shared<hardware::driver::low_spindles_pwm_fake>();
    objs.timers = std::make_shared<hardware::driver::low_timers_fake>();
    objs.motor_layout = hardware::motor_layout::get_instance(actuators_cfg);
    objs.configuration.motion_layout = actuators_cfg.motion_layout;
    objs.configuration.scale = actuators_cfg.scale;
    objs.configuration.steppers = actuators_cfg.steppers;
    objs.stepping = std::make_shared<hardware::stepping_sim>(steps_t{0, 0, 0, 0});


    hardware::driver::inmem* inmem_ptr = (hardware::driver::inmem*)objs.steppers.get();
    hardware::driver::low_timers_fake* low_timers_fake_ptr = (hardware::driver::low_timers_fake*)objs.timers.get();
    hardware::driver::low_spindles_pwm_fake* low_spindles_pwm_fake_ptr = (hardware::driver::low_spindles_pwm_fake*)objs.spindles_pwm.get();

    SECTION("set_gcode_interpreter_objects")
    {
        auto nobjs = executor.get_gcode_interpreter_objects();
        REQUIRE(objs.buttons.get() != nobjs.buttons.get());
        REQUIRE(objs.steppers.get() != nobjs.steppers.get());
        REQUIRE(objs.spindles_pwm.get() != nobjs.spindles_pwm.get());
        REQUIRE(objs.timers.get() != nobjs.timers.get());
        REQUIRE(objs.motor_layout.get() != nobjs.motor_layout.get());
        REQUIRE(objs.stepping.get() != nobjs.stepping.get());
        REQUIRE(!(objs.configuration == nobjs.configuration));

        executor.set_gcode_interpreter_objects(objs);

        nobjs = executor.get_gcode_interpreter_objects();
        REQUIRE(objs.buttons.get() == nobjs.buttons.get());
        REQUIRE(objs.steppers.get() == nobjs.steppers.get());
        REQUIRE(objs.spindles_pwm.get() == nobjs.spindles_pwm.get());
        REQUIRE(objs.timers.get() == nobjs.timers.get());
        REQUIRE(objs.motor_layout.get() == nobjs.motor_layout.get());
        REQUIRE(objs.stepping.get() == nobjs.stepping.get());
        REQUIRE(objs.configuration.max_accelerations_mm_s2 == nobjs.configuration.max_accelerations_mm_s2);
        REQUIRE(objs.configuration.max_no_accel_velocity_mm_s == nobjs.configuration.max_no_accel_velocity_mm_s);
        REQUIRE(objs.configuration.max_velocity_mm_s == nobjs.configuration.max_velocity_mm_s);


        REQUIRE(objs.configuration == nobjs.configuration);
    }


    SECTION("execute empty program. The position should not move. The task should be returned")
    {
        executor.set_gcode_interpreter_objects(objs);
        auto result = executor.execute({});
        REQUIRE(result.position == steps_t{0, 0, 0, 0});
    }

    SECTION("stepper motors enable and disable feature should work for enabling and disabling motors")
    {
        executor.set_gcode_interpreter_objects(objs);
        inmem_ptr->enabled = {false, false, false, false};
        auto result = executor.execute({movement::path_intentions::motor_t{.delay_s = 0.01, .motor = {true, true, true, true}}});
        REQUIRE(inmem_ptr->enabled == std::vector<bool>{true, true, true, true});
        result = executor.execute({movement::path_intentions::motor_t{.delay_s = 0.01, .motor = {false, false, false, false}}});
        REQUIRE(inmem_ptr->enabled == std::vector<bool>{false, false, false, false});
    }

    SECTION("stepper motors enable and disable feature should work with delay")
    {
        executor.set_gcode_interpreter_objects(objs);
        inmem_ptr->enabled = {false, false, false, false};
        auto result = executor.execute({movement::path_intentions::motor_t{.delay_s = 0.01, .motor = {true, true, true, true}}});
        REQUIRE(low_timers_fake_ptr->last_delay == Approx(0.01));
        result = executor.execute({movement::path_intentions::motor_t{.delay_s = 0.02, .motor = {false, false, false, false}}});
        REQUIRE(low_timers_fake_ptr->last_delay == Approx(0.02));
    }

    SECTION("stepper motors enable and disable feature should work with delay and in order")
    {
        std::list<int> rec;
        executor.set_gcode_interpreter_objects(objs);
        inmem_ptr->on_enable_steppers = [&rec](auto) { rec.push_back(0); };
        low_timers_fake_ptr->on_wait_s = [&rec](auto) { rec.push_back(1); };
        auto result = executor.execute({movement::path_intentions::motor_t{.delay_s = 0.01, .motor = {true, true, true, true}}});
        REQUIRE(rec == std::list<int>{0, 1});
        result = executor.execute({movement::path_intentions::motor_t{.delay_s = 0.02, .motor = {false, false, false, false}}});
        REQUIRE(rec == std::list<int>{0, 1, 0, 1});
        result = executor.execute({movement::path_intentions::motor_t{.delay_s = 0.01, .motor = {true, true, true, true}}, movement::path_intentions::motor_t{.delay_s = 0.02, .motor = {false, false, false, false}}});
        REQUIRE(rec == std::list<int>{0, 1, 0, 1, 0, 1, 0, 1});
    }


    SECTION("delay command test")
    {
        executor.set_gcode_interpreter_objects(objs);
        std::list<double> rec;

        low_timers_fake_ptr->on_wait_s = [&rec](const double e) { rec.push_back(e); };
        auto result = executor.execute({movement::path_intentions::pause_t{.delay_s = 0.1},
            movement::path_intentions::pause_t{.delay_s = 0.2},
            movement::path_intentions::pause_t{.delay_s = 0.3},
            movement::path_intentions::pause_t{.delay_s = 0.4}});
        REQUIRE(low_timers_fake_ptr->last_delay == Approx(0.4));
        REQUIRE(rec == std::list<double>{0.1, 0.2, 0.3, 0.4});
    }


    SECTION("spindle enable and disable feature should work for enabling and disabling motors")
    {
        executor.set_gcode_interpreter_objects(objs);
        auto result = executor.execute({movement::path_intentions::spindle_t{.delay_s = 0.01, .spindle = {{0, 1.0}}},
            movement::path_intentions::spindle_t{.delay_s = 0.01, .spindle = {{1, 0.5}}}});
        REQUIRE(low_spindles_pwm_fake_ptr->spindle_values[0] == Approx(1.0));
        REQUIRE(low_spindles_pwm_fake_ptr->spindle_values[1] == Approx(0.5));
        REQUIRE(low_timers_fake_ptr->last_delay == Approx(0.01));
        result = executor.execute({movement::path_intentions::spindle_t{.delay_s = 0.1, .spindle = {{1, 0.0}}}});
        REQUIRE(low_spindles_pwm_fake_ptr->spindle_values[1] == Approx(0.0));
        REQUIRE(low_timers_fake_ptr->last_delay == Approx(0.1));
    }

    SECTION("stepper motors enable and disable feature should work with delay and in order")
    {
        std::list<int> rec;
        executor.set_gcode_interpreter_objects(objs);
        inmem_ptr->on_enable_steppers = [&rec](auto) { rec.push_back(0); };
        low_timers_fake_ptr->on_wait_s = [&rec](auto) { rec.push_back(1); };
        low_spindles_pwm_fake_ptr->on_spindle_pwm_power = [&rec](auto, auto) { rec.push_back(2); };
        auto result = executor.execute({movement::path_intentions::spindle_t{.delay_s = 0.01, .spindle = {{0, 1.0}}},
            movement::path_intentions::pause_t{.delay_s = 0.2},
            movement::path_intentions::spindle_t{.delay_s = 0.01, .spindle = {{1, 0.5}}}});
        REQUIRE(rec == std::list<int>{2, 1, 1, 2, 1});
    }
}
