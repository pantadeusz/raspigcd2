#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>
#include <converters/gcd_program_to_steps.hpp>
#include <hardware/stepping_commands.hpp>
#include <hardware/stepping.hpp>
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
    steps_t steps_per_mm_arr;
    for (int i = 0; i < RASPIGCD_HARDWARE_DOF; i++) {
        steps_per_mm_arr[i] = test_config.steppers[i].steps_per_mm;
    };

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
           G1X1F10
        )");
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()) );
        //REQUIRE(result.size() == 200); // empty+step * 100
        steps_t steps = {0,0,0,0};
        int commands_count = 0; 
        for (auto &e : result) {
            for (int i = 0; i < e.count; i++) {
                commands_count++;
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
           G1F1
           G1X1F1
        )");
        // 1mm/s, 
        // 1s is  1000000/test_config.tick_duration_us -> this is the time of the movement
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()) );
        //REQUIRE(result.size() == 200); // empty+step * 100
        steps_t steps = {0,0,0,0};
        int commands_count = 0; 
        for (auto &e : result) {
            for (int i = 0; i < e.count; i++) {
                commands_count++;
                for (int i = 0; i < RASPIGCD_HARDWARE_DOF; i++) {
                    auto m = e.b[i];
                    //std::cout << "s:" << m.step << " " << m.dir;
                    if (m.step) steps[i] += ((int)(m.dir)*2)-1;
                }
                //std::cout << std::endl;
            }
        }
        REQUIRE(steps == steps_t{100,0,0,0});
        REQUIRE(commands_count == (1000000/test_config.tick_duration_us));
    }
    SECTION("if the speed is 0 and the distance is not 0, then the exception should be throwned")
    {
        auto program = gcode_to_maps_of_arguments(R"(
           G1F0
           G1X1F0
        )");
        REQUIRE_THROWS( program_to_steps(program,test_config, *(motor_layot_p.get()) ));
    }
    SECTION("acceleration from F0 to F1 should result in correct distance")
    {
        auto program = gcode_to_maps_of_arguments(R"(
           G1F0
           G1X1F1
        )");
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()) );
        steps_t steps = {0,0,0,0};
        int commands_count = 0; 
        for (auto &e : result) {
            for (int i = 0; i < e.count; i++) {
                commands_count++;
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
           G1F0
           )" +
           std::string("G1X") + std::to_string(s) + "F" + std::to_string(v1)
        );
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()) );
        steps_t steps = {0,0,0,0};
        int commands_count = 0; 
        for (auto &e : result) {
            for (int i = 0; i < e.count; i++) {
                commands_count++;
                for (int i = 0; i < RASPIGCD_HARDWARE_DOF; i++) {
                    auto m = e.b[i];
                    if (m.step) steps[i] += ((int)(m.dir)*2)-1;
                }
            }
        }
        REQUIRE(steps == steps_t{(int)(s*100),0,0,0});
        double dt = ((double) test_config.tick_duration_us)/1000000.0;
        REQUIRE(commands_count == (int)(t/dt));
//        REQUIRE(result.size() == (1000000/test_config.tick_duration_us));
    }
    SECTION("break from F1 to F0 should result in correct time")
    {
        double t = 1;
        double a = -10;
        double v0 = 10;
        double v1 = v0 + a*t;
        v1 = 0;
        double s = v0 + a*t*t/2.0;
        auto program = gcode_to_maps_of_arguments(
           std::string("G1X") + std::to_string(0) + "F" + std::to_string(v0) + "\n" +
           std::string("G1X") + std::to_string(s) + "F" + std::to_string(v1)
        );
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()) );
        steps_t steps = {0,0,0,0};
        int commands_count = 0; 
        for (auto &e : result) {
            for (int i = 0; i < e.count; i++) {
                commands_count++;
                for (int i = 0; i < RASPIGCD_HARDWARE_DOF; i++) {
                    auto m = e.b[i];
                    if (m.step) steps[i] += ((int)(m.dir)*2)-1;
                }
            }
        }
        REQUIRE(steps == steps_t{(int)(s*100),0,0,0});
        double dt = ((double) test_config.tick_duration_us)/1000000.0;
        REQUIRE(commands_count == (int)(t/dt));
    }
    SECTION("program_to_steps should optimize the commands count - reduce it")
    {
        double t = 1;
        double a = -10;
        double v0 = 10;
        double v1 = v0 + a*t;
        v1 = 0;
        double s = v0 + a*t*t/2.0;
        auto program = gcode_to_maps_of_arguments(
           std::string("G1X") + std::to_string(0) + "F" + std::to_string(v0) + "\n" +
           std::string("G1X") + std::to_string(s) + "F" + std::to_string(v1)
        );
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()) );
        steps_t steps = {0,0,0,0};
        int commands_count = 0; 
        for (auto &e : result) {
            for (int i = 0; i < e.count; i++) {
                commands_count++;
                for (int i = 0; i < RASPIGCD_HARDWARE_DOF; i++) {
                    auto m = e.b[i];
                    if (m.step) steps[i] += ((int)(m.dir)*2)-1;
                }
            }
        }
        REQUIRE(commands_count > result.size() );
    }
    
    SECTION("end state verification")
    {
        auto program = gcode_to_maps_of_arguments(R"(
           G1F0
           G1X1F1
           G1X10Y20Z10F4
           G1X2Y2Z3A4
           G1X1
        )");
        block_t machine_state = {{'F',1}};
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()),
        machine_state, [&machine_state](const block_t &result){
            machine_state = result;
        } );

        steps_t steps = {0,0,0,0};
        int commands_count = 0; 
        for (auto &e : result) {
            for (int i = 0; i < e.count; i++) {
                commands_count++;
                for (int i = 0; i < RASPIGCD_HARDWARE_DOF; i++) {
                    auto m = e.b[i];
                    if (m.step) steps[i] += ((int)(m.dir)*2)-1;
                }
            }
        }

        steps_t expected_steps = steps_per_mm_arr*steps_t{1,2,3,4};
        REQUIRE(steps == expected_steps);
        std::list<steps_t> hstps = hardware_commands_to_steps(result);
        REQUIRE(hstps.back() == expected_steps);

        //double dt = ((double) test_config.tick_duration_us)/1000000.0;
        //REQUIRE(commands_count == (int)(t/dt));


        REQUIRE(machine_state['F'] == Approx(4));
        REQUIRE(machine_state['X'] == Approx(1));
        REQUIRE(machine_state['Y'] == Approx(2));
        REQUIRE(machine_state['Z'] == Approx(3));
        REQUIRE(machine_state['A'] == Approx(4));
        auto ls = last_state_after_program_execution(program, {{'F',1}});
        REQUIRE(ls == machine_state);
    }

    SECTION("accept coordinates change via G92") {
        auto program = gcode_to_maps_of_arguments(R"(
           G1F0
           G1X1F1
           G1X10Y20Z10F4
           G1X0Y0Z0F4
           G92X101Y102Z103A104
           G1X2Y2Z3A4
           G1X1
        )");
        block_t machine_state = {{'F',1}};
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()),
        machine_state, [&machine_state](const block_t &result){
            machine_state = result;
        } );
        REQUIRE(machine_state['F'] == Approx(4));
        REQUIRE(machine_state['X'] == Approx(1));
        REQUIRE(machine_state['Y'] == Approx(2));
        REQUIRE(machine_state['Z'] == Approx(3));
        REQUIRE(machine_state['A'] == Approx(4));
        auto ls = last_state_after_program_execution(program, {{'F',1}});
        REQUIRE(ls == machine_state);

        steps_t expected_steps = steps_per_mm_arr*steps_t{-100,-100,-100,-100};
        std::list<steps_t> hstps = hardware_commands_to_steps(result);
        REQUIRE(hstps.back() == expected_steps);

    } 
}
