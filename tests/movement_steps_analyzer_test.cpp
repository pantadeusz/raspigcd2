#include <configuration.hpp>
#include <configuration_json.hpp>
#include <hardware/stepping.hpp>
#include <movement/variable_speed.hpp>
#include <movement/steps_analyzer.hpp>
#include <hardware/driver/inmem.hpp>

#define CATCH_CONFIG_MAIN
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


SCENARIO( "steps analyzer", "[movement][steps_analyzer]" ) {
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

	path_intent_t simple_program = {
		distance_t{0, 0, 0, 0}, 8.0,
		distance_t{10, 0, 0, 0}
	};
	auto plan_to_execute = variable_speed_driver.intent_to_movement_plan( simple_program );
    steps_t psteps = {0, 0, 0, 0};
  
	// steps analyzer - we test it
	steps_analyzer sa(motor_layout_);


	GIVEN( "we have ticks to execute" ) {
		std::vector<hardware::multistep_command> ticks_to_execute;
		steps_generator.movement_plan_to_step_commands(plan_to_execute, cfg.tick_duration(), 
			[&psteps,&stepping,&ticks_to_execute](std::unique_ptr<std::vector<hardware::multistep_command> > msteps_p){
				ticks_to_execute.insert(ticks_to_execute.end(), msteps_p.get()->begin(), msteps_p.get()->end());
			});

		WHEN( "we calculate every step" ) {
			std::list<steps_t> steps_in_order = hardware_commands_to_steps(ticks_to_execute);

			THEN( "steps_from_tick are calculated the same as in hardware_commands_to_steps" ) {
				int i = 0;
				for (auto &tt: steps_in_order) {
					i++;
					steps_t val = sa.steps_from_tick(ticks_to_execute, i);
					REQUIRE( tt == val );
				}
			}
		//     int get_last_tick_index(const std::vector<hardware::multistep_command> &commands_to_do) const {

			THEN( "the number of ticks should be equal to get_last_tick_index" ) {
				REQUIRE( steps_in_order.size() == sa.get_last_tick_index(ticks_to_execute) );
			}

		}
	}
}
