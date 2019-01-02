#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>
#include <converters/gcd_program_to_steps.hpp>
#include <hardware/stepping_commands.hpp>
#include <gcd/gcode_interpreter.hpp>

#include <thread>
#include <vector>

using namespace raspigcd;
using namespace raspigcd::gcd;
using namespace raspigcd::converters;

TEST_CASE("converters - program_to_steps", "[gcd][converters][program_to_steps]")
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

    auto motor_layot_p = hardware::motor_layout::get_instance(test_config);
    motor_layot_p->set_configuration(test_config);
    //std::cout << "sdfdss::"<<(motor_layot_p.get()->cartesian_to_steps({1,2,3,4})) << "   <<<)))))_____  \n";
    SECTION("empty program should result in empty steps list")
    {
        REQUIRE(program_to_steps({},test_config, *(motor_layot_p.get()) ) .size() == 0);
    }
    SECTION("simple move along x axis for 100 steps should result in correct distance generated")
    {
        auto program = gcode_to_maps_of_arguments(R"(
           G0X1F10
        )");
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()) );
        //REQUIRE(result.size() == 200); // empty+step * 100
        steps_t steps = {0,0,0,0};
        for (auto &e : result) {
            for (int i = 0; i < e.count; i++) {
                //std::cout << "m: ";
                for (int i = 0; i < RASPIGCD_HARDWARE_DOF; i++) {
                    auto m = e.b[i];
                    //std::cout << "s:" << m.step << " " << m.dir;
                    if (m.step) steps[i] += ((int)(m.dir)*2)-1;
                }
                //std::cout << std::endl;
            }
        }
        REQUIRE(steps == steps_t{100,0,0,0});
    }
    SECTION("simple move along x axis for 100 steps should result in correct speed")
    {
        auto program = gcode_to_maps_of_arguments(R"(
           G0F1
           G0X1F1
        )");
        // 1mm/s, 
        // 1s is  1000000/test_config.tick_duration_us -> this is the time of the movement
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()) );
        //REQUIRE(result.size() == 200); // empty+step * 100
        steps_t steps = {0,0,0,0};
        for (auto &e : result) {
            for (int i = 0; i < e.count; i++) {
                //std::cout << "m: ";
                for (int i = 0; i < RASPIGCD_HARDWARE_DOF; i++) {
                    auto m = e.b[i];
                    //std::cout << "s:" << m.step << " " << m.dir;
                    if (m.step) steps[i] += ((int)(m.dir)*2)-1;
                }
                //std::cout << std::endl;
            }
        }
        REQUIRE(steps == steps_t{100,0,0,0});
        REQUIRE(result.size() == (1000000/test_config.tick_duration_us));
    }
    SECTION("if the speed is 0 and the distance is not 0, then the exception should be throwned")
    {
        auto program = gcode_to_maps_of_arguments(R"(
           G0F0
           G0X1F0
        )");
        REQUIRE_THROWS( program_to_steps(program,test_config, *(motor_layot_p.get()) ));
    }
    SECTION("acceleration from F0 to F1 should result in correct distance")
    {
        auto program = gcode_to_maps_of_arguments(R"(
           G0F0
           G0X1F1
        )");
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()) );
        steps_t steps = {0,0,0,0};
        for (auto &e : result) {
            for (int i = 0; i < e.count; i++) {
                for (int i = 0; i < RASPIGCD_HARDWARE_DOF; i++) {
                    auto m = e.b[i];
                    if (m.step) steps[i] += ((int)(m.dir)*2)-1;
                }
            }
        }
        REQUIRE(steps == steps_t{100,0,0,0});
    }

    SECTION("acceleration from F0 to F1 should result in correct time")
    {
        double t = 1;
        double a = 100;
        double s = a*t*t/2.0;
        double v1 = a*t;
        auto program = gcode_to_maps_of_arguments(R"(
           G0F0
           )" +
           std::string("G0X") + std::to_string(s) + "F" + std::to_string(v1)
        );
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()) );
        steps_t steps = {0,0,0,0};
        for (auto &e : result) {
            for (int i = 0; i < e.count; i++) {
                for (int i = 0; i < RASPIGCD_HARDWARE_DOF; i++) {
                    auto m = e.b[i];
                    if (m.step) steps[i] += ((int)(m.dir)*2)-1;
                }
            }
        }
        REQUIRE(steps == steps_t{(int)(s*100),0,0,0});
        double dt = ((double) test_config.tick_duration_us)/1000000.0;
        REQUIRE(result.size() == (int)(t/dt));
//        REQUIRE(result.size() == (1000000/test_config.tick_duration_us));
    }
    
    //SECTION("program that stays in the same place should result in empty result")
    //{
    //}
}
