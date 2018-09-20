#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>


#include <executor_t.hpp>
#include <vector>

using namespace raspigcd;

TEST_CASE( "Steps calculated by executor_t", "[executor_t]" ) {


    SECTION( "trivial test - no steps" ) {
        std::vector<executor_command_t> commands;
        steps_t steps = executor_t::commands_to_steps(commands);

        REQUIRE( steps[0] == 0 );
        REQUIRE( steps[1] == 0 );
        REQUIRE( steps[2] == 0 );
        REQUIRE( steps[3] == 0 );
    }
    SECTION( "ones on each direction" ) {
        std::vector<executor_command_t> commands;
        commands.push_back({b:{{step:1,dir:1},{step:1,dir:1},{step:1,dir:0},{step:1,dir:0}}});
        steps_t steps = executor_t::commands_to_steps(commands);

        REQUIRE( steps[0] == 1 );
        REQUIRE( steps[1] == 1 );
        REQUIRE( steps[2] == -1 );
        REQUIRE( steps[3] == -1 );
    }
    SECTION( "go back and forward" ) {
        std::vector<executor_command_t> commands;
        commands.push_back({b:{{step:1,dir:1},{step:1,dir:1},{step:1,dir:0},{step:1,dir:0}}});
        commands.push_back({b:{{step:1,dir:1},{step:1,dir:1},{step:1,dir:0},{step:1,dir:0}}});
        commands.push_back({b:{{step:1,dir:0},{step:1,dir:0},{step:1,dir:1},{step:1,dir:1}}});
        commands.push_back({b:{{step:1,dir:0},{step:1,dir:0},{step:1,dir:1},{step:1,dir:1}}});

        steps_t steps = executor_t::commands_to_steps(commands);

        REQUIRE( steps[0] == 0 );
        REQUIRE( steps[1] == 0 );
        REQUIRE( steps[2] == 0 );
        REQUIRE( steps[3] == 0 );
    }
}
