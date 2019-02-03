#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>
#include <converters/gcd_program_to_raw_gcd.hpp>
#include <hardware/stepping_commands.hpp>
#include <hardware/stepping.hpp>
#include <gcd/gcode_interpreter.hpp>

#include <thread>
#include <vector>

using namespace raspigcd;
using namespace raspigcd::gcd;
using namespace raspigcd::converters;

TEST_CASE("converters - program_to_raw_program", "[gcd][converters][program_to_raw_program]")
{
    configuration::actuators_organization test_config;

    test_config.motion_layout = configuration::motion_layouts::CARTESIAN;
    test_config.scale = {1,1,1,1};
    test_config.tick_duration_us = 100; // 0.0001 s

    for (int i = 0; i <RASPIGCD_HARDWARE_DOF;i++){
        // auto & stepper: test_config.steppers) {
        configuration::stepper stepper;
        stepper.dir = 1;
        stepper.en = 2;
        stepper.step = 3;
        stepper.steps_per_mm = 100;
        test_config.steppers.push_back(stepper);
    }
    steps_t steps_per_mm_arr;
    for (int i = 0; i < RASPIGCD_HARDWARE_DOF; i++) {
        steps_per_mm_arr[i] = test_config.steppers[i].steps_per_mm;
    };

    auto motor_layot_p = hardware::motor_layout::get_instance(test_config);
    motor_layot_p->set_configuration(test_config);
    //std::cout << "sdfdss::"<<(motor_layot_p.get()->cartesian_to_steps({1,2,3,4})) << "   <<<)))))_____  \n";
    SECTION("empty program should result in empty steps list")
    {
        REQUIRE(program_to_raw_program({},test_config, *(motor_layot_p.get()) ).size() == 0);

    }

}
