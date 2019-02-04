#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>
#include <converters/gcd_program_to_raw_gcd.hpp>
#include <hardware/stepping_commands.hpp>
#include <hardware/stepping.hpp>
#include <gcd/gcode_interpreter.hpp>
#include <gcd/gcode_interpreter.hpp>

#include <thread>
#include <vector>

using namespace raspigcd;
using namespace raspigcd::gcd;
using namespace raspigcd::converters;



TEST_CASE("converters - program_to_raw_program", "[gcd][converters][program_to_raw_program]")
{
    configuration::limits test_config;
    test_config.max_accelerations_mm_s2= {500,500,500,500};
    test_config.max_no_accel_velocity_mm_s = {1,1,1,1};
    test_config.max_velocity_mm_s = {101,102,103,104};

    //std::cout << "sdfdss::"<<(motor_layot_p.get()->cartesian_to_steps({1,2,3,4})) << "   <<<)))))_____  \n";
    SECTION("empty program should result in empty steps list")
    {
        REQUIRE(program_to_raw_program({},test_config ).size() == 0);
    }
    SECTION("program with only commands from M category goes unchanged")
    {
        program_t program = gcode_to_maps_of_arguments("M17\nM3\nM18\nM5");
        program_t result_program = program_to_raw_program(program,test_config );
        REQUIRE(program == result_program);
    }
    SECTION("text or grouped program should result in the same output")
    {
        std::string program_txt = "M17\nM3\nM18\nM5";
        program_t program = gcode_to_maps_of_arguments(program_txt);
        program_t result_program1 = program_to_raw_program_str(program_txt,test_config );
        program_t result_program2 = program_to_raw_program(program,test_config );
        REQUIRE(result_program1 == result_program2);
    }
    SECTION("g0 moevements should be optimized")
    {
        std::string program_txt = "M17\nM3\nG0X100\nM18\nG0X0\nM5";
        program_t program = gcode_to_maps_of_arguments(program_txt);
        program_t result_program = program_to_raw_program(program,test_config );
        INFO(back_to_gcode({result_program}));
        REQUIRE(!(program == result_program));
    }
    
    //SECTION("program should be cleared of unnecessary commands")
    //{
    //    std::string program_txt = "M17\nM3\nM18\nM5";
    //    program_t program = gcode_to_maps_of_arguments(program_txt);
    //    program_t result_program1 = program_to_raw_program_str(program_txt,test_config );
    //    program_t result_program2 = program_to_raw_program(program,test_config );
    //    REQUIRE(result_program1 == result_program2);
    //}
    
}
