#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>
#include <chrono>
#include <gcd/path_intent_executor.hpp>
#include <hardware/driver/inmem.hpp>
#include <hardware/driver/low_spindles_pwm_fake.hpp>
#include <hardware/driver/low_timers_fake.hpp>
#include <hardware/driver/low_buttons_fake.hpp>
#include <hardware/stepping.hpp>
#include <thread>
#include <vector>

using namespace raspigcd;
using namespace raspigcd::movement;
using namespace raspigcd::configuration;


TEST_CASE("path_intent_executor constructor tests", "[gcd][path_intent_executor]")
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
        REQUIRE(objs.configuration == nobjs.configuration);
    }
}
