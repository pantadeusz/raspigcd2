#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_FAST_COMPILE
#define CATCH_CONFIG_DISABLE_MATCHERS
#include <catch2/catch.hpp>


#include <step_sequence_executor_t.hpp>
#include <vector>

using namespace raspigcd;

TEST_CASE( "Steps calculated by step_sequence_executor_t", "[step_sequence_executor_t]" ) {


    SECTION( "trivial test - no steps" ) {
        std::vector<executor_command_t> commands;
        steps_t steps = step_sequence_executor_t::commands_to_steps(commands);

        REQUIRE( steps[0] == 0 );
        REQUIRE( steps[1] == 0 );
        REQUIRE( steps[2] == 0 );
        REQUIRE( steps[3] == 0 );
    }
    SECTION( "ones on each direction" ) {
        std::vector<executor_command_t> commands;
        executor_command_t cmnd = {cmnd:{b:{{step:1,dir:1},{step:1,dir:1},{step:1,dir:0},{step:1,dir:0}}, repeat:0 }};
        commands.push_back(cmnd);
        steps_t steps = step_sequence_executor_t::commands_to_steps(commands);

        REQUIRE( steps[0] == 1 );
        REQUIRE( steps[1] == 1 );
        REQUIRE( steps[2] == -1 );
        REQUIRE( steps[3] == -1 );
    }
    SECTION( "go back and forward" ) {
        std::vector<executor_command_t> commands;
        
        executor_command_t cmnd = {cmnd:{b:{{step:1,dir:1},{step:1,dir:1},{step:1,dir:0},{step:1,dir:0}}, repeat:0}};
        commands.push_back(cmnd);
        cmnd = {cmnd:{b:{{step:1,dir:1},{step:1,dir:1},{step:1,dir:0},{step:1,dir:0}}, repeat:0}};
        commands.push_back(cmnd);
        cmnd = {cmnd:{b:{{step:1,dir:0},{step:1,dir:0},{step:1,dir:1},{step:1,dir:1}}, repeat:0}};
        commands.push_back(cmnd);
        cmnd = {cmnd:{b:{{step:1,dir:0},{step:1,dir:0},{step:1,dir:1},{step:1,dir:1}}, repeat:0}};
        commands.push_back(cmnd);

        steps_t steps = step_sequence_executor_t::commands_to_steps(commands);

        REQUIRE( steps[0] == 0 );
        REQUIRE( steps[1] == 0 );
        REQUIRE( steps[2] == 0 );
        REQUIRE( steps[3] == 0 );
    }
    SECTION( "go back and forward with steps count" ) {
        std::vector<executor_command_t> commands;
        
        executor_command_t cmnd = {cmnd:{b:{{step:1,dir:1},{step:1,dir:1},{step:1,dir:0},{step:1,dir:0}}, repeat:0}};
        commands.push_back(cmnd);
        cmnd = {cmnd:{b:{{step:1,dir:1},{step:1,dir:1},{step:1,dir:0},{step:1,dir:0}}, repeat:0}};
        commands.push_back(cmnd);
        cmnd = {cmnd:{b:{{step:1,dir:0},{step:1,dir:0},{step:1,dir:1},{step:1,dir:1}}, repeat:2}};
        commands.push_back(cmnd);

        steps_t steps = step_sequence_executor_t::commands_to_steps(commands);

        REQUIRE( steps[0] == -1 );
        REQUIRE( steps[1] == -1 );
        REQUIRE( steps[2] == 1 );
        REQUIRE( steps[3] == 1 );
    }
}
