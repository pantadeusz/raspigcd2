#include <configuration.hpp>
#include <configuration_json.hpp>
#include <hardware/stepping.hpp>
#include <movement/variable_speed.hpp>
#include <hardware/driver/inmem.hpp>

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

SCENARIO( "variable speed and accelerations", "[movement][variable_speed]" ) {
    std::shared_ptr<low_steppers> lsfake = std::make_shared<driver::inmem>();
	configuration::global cfg;
	cfg.load_defaults();
	double acceleration = 100;
	double max_speed_no_accel = 5;
	cfg.tick_duration_us = 60;
	cfg.max_no_accel_velocity_mm_s = {max_speed_no_accel, max_speed_no_accel, max_speed_no_accel, max_speed_no_accel};
	cfg.max_accelerations_mm_s2 = {acceleration,acceleration,acceleration,acceleration};
	cfg.max_velocity_mm_s = {150,150,150,150};
	std::shared_ptr<motor_layout> motor_layout_ = motor_layout::get_instance( cfg );
	movement::variable_speed variable_speed_driver( motor_layout_, cfg, cfg.tick_duration() );
	stepping_sim stepping( {0,0,0,0} );
	movement::steps_generator steps_generator( motor_layout_ );

	//distance_t start_coord = {0, 0, 0, 0};
	//steps_t steps = motor_layout_.get()->cartesian_to_steps(start_coord);
	//double velocity = 30; // mm/s

	GIVEN( "there is an empty program to run" ) {
		movement::path_intent_t empty_program;
		WHEN( "the program is converted to actions" ) {
			std::list<movement_plan_element_t>
					result = variable_speed_driver.intent_to_movement_plan( empty_program );
			THEN( "actions should be empty" ) {
				REQUIRE( result.size() == 0 );
			}
		}
	}

	GIVEN( "there is a program with constant and safe speed" ) {
		movement::path_intent_t simple_program;
		simple_program.push_back( distance_t{0, 0, 0, 0} );
		simple_program.push_back( 2 );
		simple_program.push_back( distance_t{1, 0, 0, 0} );
		simple_program.push_back( 2 );
		simple_program.push_back( distance_t{1, 1, 0, 0} );
		simple_program.push_back( 2 );
		simple_program.push_back( distance_t{0, 1, 0, 0} );
		simple_program.push_back( 2 );
		simple_program.push_back( distance_t{0, 0, 0, 0} );
		std::vector<movement::path_intent_element_t> simple_program_v( simple_program.begin(), simple_program.end() );

		WHEN( "the program is converted to actions" ) {
			std::list<movement_plan_element_t>
					result = variable_speed_driver.intent_to_movement_plan( simple_program );
			std::vector<movement_plan_element_t> result_v( result.begin(), result.end() );
			THEN( "the result should be of the same size as simple program" ) {
				REQUIRE( result.size() == simple_program.size() );
			}
			THEN( "the result should have correct coordinates" ) {
				for ( unsigned i = 0; i < result_v.size(); i += 2 ) {
					REQUIRE( std::get<distance_t>( result_v[i] ) == std::get<distance_t>( simple_program_v[i] ) );
				}
			}
			THEN( "the result should have correct maximal and actual speeds" ) {
				for ( unsigned i = 1; i < result_v.size(); i += 2 ) {
					REQUIRE( std::get<transition_t>( result_v[i] ).v0 == std::get<double>( simple_program_v[i] ) );
					REQUIRE( std::get<transition_t>( result_v[i] ).max_v == std::get<double>( simple_program_v[i] ) );
					REQUIRE( std::get<transition_t>( result_v[i] ).accel == 0.0 );
				}
			}
		}
	}

	GIVEN( "there is a program with different and safe speed" ) {
		movement::path_intent_t simple_program = {
			distance_t{0, 0, 0, 0}, 2.0,
			distance_t{1, 0, 0, 0}, 1.0,
			distance_t{1, 1, 0, 0}, 2.0,
			distance_t{0, 1, 0, 0}, 3.0,
			distance_t{0, 0, 0, 0}
		};
		std::vector<movement::path_intent_element_t> simple_program_v( simple_program.begin(), simple_program.end() );

		WHEN( "the program is converted to actions" ) {
			std::list<movement_plan_element_t>
					result = variable_speed_driver.intent_to_movement_plan( simple_program );
			std::vector<movement_plan_element_t> result_v( result.begin(), result.end() );
			THEN( "the result should be of the same size as simple program" ) {
				REQUIRE( result.size() == simple_program.size() );
			}
			THEN( "the result should have correct coordinates" ) {
				for ( unsigned i = 0; i < result_v.size(); i += 2 ) {
					REQUIRE( std::get<distance_t>( result_v[i] ) == std::get<distance_t>( simple_program_v[i] ) );
				}
			}
			THEN( "the result should have correct maximal and actual speeds" ) {
				for ( unsigned i = 1; i < result_v.size(); i += 2 ) {
					REQUIRE( std::get<transition_t>( result_v[i] ).v0 == std::get<double>( simple_program_v[i] ) );
					REQUIRE( std::get<transition_t>( result_v[i] ).max_v == std::get<double>( simple_program_v[i] ) );
					REQUIRE( std::get<transition_t>( result_v[i] ).accel == 0.0 );
				}
			}
			THEN( "actions should result in going back to the origin" ) {
				steps_t steps = {0, 0, 0, 0};
				stepping.current_steps = steps;
				for ( unsigned i = 2; i < result_v.size(); i += 2 ) {
					auto commands = steps_generator.movement_from_to( std::get<distance_t>( result_v[i - 2] ), std::get<transition_t>( result_v[i - 1] ), std::get<distance_t>( result_v[i] ), cfg.tick_duration() );
					stepping.exec( commands );
					steps = stepping.current_steps;
				}
				distance_t p1_expected = {0, 0, 0, 0};
				steps_t expected_steps = motor_layout_.get()->cartesian_to_steps( p1_expected );
				REQUIRE( steps == expected_steps );
			}
		}
	}
	GIVEN( "there is a program that needs acceleration and possible to reach speed" ) {
		movement::path_intent_t simple_program = {
			distance_t{0, 0, 0, 0}, 8.0,
			distance_t{10, 0, 0, 0}
		};
		std::vector<movement::path_intent_element_t> simple_program_v( simple_program.begin(), simple_program.end() );

		WHEN( "the program is converted to actions" ) {
			std::list<movement_plan_element_t>
					result = variable_speed_driver.intent_to_movement_plan( simple_program );
			std::vector<movement_plan_element_t> result_v( result.begin(), result.end() );
			THEN( "the result should be of same 7" ) {
				REQUIRE( result.size() == 7 );
			}
			THEN( "the result should have correct coordinates at the beginning and the end" ) {
				REQUIRE( std::get<distance_t>( result_v[0] ) == std::get<distance_t>( simple_program_v[0] ) );
				REQUIRE( std::get<distance_t>( result_v[6] ) == std::get<distance_t>( simple_program_v[2] ) );
			}
			THEN( "the result path should sum up to the same lenght as intended path" ) {
				double full_length = get_path_length( simple_program );
				double full_length_of_sum = get_path_length( result );
				REQUIRE( full_length == Approx( full_length_of_sum ) );
			}
			THEN( "the result path should have proper accelerations" ) {
				REQUIRE( std::get<transition_t>( result_v[1] ).accel == acceleration );
				REQUIRE( std::get<transition_t>( result_v[3] ).accel == 0 );
				REQUIRE( std::get<transition_t>( result_v[5] ).accel == -acceleration );
			}
			THEN( "the result path should have proper initial speeds" ) {
				REQUIRE( std::get<transition_t>( result_v[1] ).v0 == max_speed_no_accel );
				REQUIRE( std::get<transition_t>( result_v[3] ).v0 == 8.0 );
				REQUIRE( std::get<transition_t>( result_v[5] ).v0 == 8.0 );
			}
			THEN( "the result path should have proper max speeds" ) {
				REQUIRE( std::get<transition_t>( result_v[1] ).max_v == 8.0 );
				REQUIRE( std::get<transition_t>( result_v[3] ).max_v == 8.0 );
				REQUIRE( std::get<transition_t>( result_v[5] ).max_v == 8.0 );
			}
			THEN( "actions should be possible to execute" ) {
				steps_t steps = {0, 0, 0, 0};
				stepping.current_steps = steps;
				for ( unsigned i = 2; i < result_v.size(); i += 2 ) {
					auto commands = steps_generator.movement_from_to( std::get<distance_t>( result_v[i - 2] ), std::get<transition_t>( result_v[i - 1] ), std::get<distance_t>( result_v[i] ), cfg.tick_duration() );
					stepping.exec(commands);
					steps = stepping.current_steps;
				}
				distance_t p1_expected = {10, 0, 0, 0};
				steps_t expected_steps = motor_layout_.get()->cartesian_to_steps( p1_expected );
				REQUIRE( steps == expected_steps );
			}
		}
	}
	GIVEN( "there is a program with acceleration and impossible to reach speed" ) {
		movement::path_intent_t simple_program = {
			distance_t{0, 0, 0, 0}, 16.0,
			distance_t{1, 0, 0, 0}
		};
		std::vector<movement::path_intent_element_t> simple_program_v( simple_program.begin(), simple_program.end() );

		WHEN( "the program is converted to actions" ) {
			std::list<movement_plan_element_t>
					result = variable_speed_driver.intent_to_movement_plan( simple_program );
			std::vector<movement_plan_element_t> result_v( result.begin(), result.end() );
			THEN( "the result should be of the same size as simple program" ) {
				REQUIRE( result.size() == simple_program.size() );
			}
			THEN( "the result should have correct coordinates" ) {
				for ( unsigned i = 0; i < result_v.size(); i += 2 ) {
					REQUIRE( std::get<distance_t>( result_v[i] ) == std::get<distance_t>( simple_program_v[i] ) );
				}
			}
			THEN( "the result should have correct maximal and actual speeds" ) {
				for ( unsigned i = 1; i < result_v.size(); i += 2 ) {
					REQUIRE( std::get<transition_t>( result_v[i] ).v0 == max_speed_no_accel );
					REQUIRE( std::get<transition_t>( result_v[i] ).max_v == std::get<double>( simple_program_v[i] ) );
					REQUIRE( std::get<transition_t>( result_v[i] ).accel == 0.0 );
				}
			}
			THEN( "actions should be possible to execute" ) {
				steps_t steps = {0, 0, 0, 0};
				stepping.current_steps = steps;
				for ( unsigned i = 2; i < result_v.size(); i += 2 ) {
					auto commands = steps_generator.movement_from_to( std::get<distance_t>( result_v[i - 2] ), std::get<transition_t>( result_v[i - 1] ), std::get<distance_t>( result_v[i] ), cfg.tick_duration() );
					stepping.exec(commands);
					steps = stepping.current_steps;
				}
				distance_t p1_expected = {1, 0, 0, 0};
				steps_t expected_steps = motor_layout_.get()->cartesian_to_steps( p1_expected );
				REQUIRE( steps == expected_steps );
			}
		}
	}
}
