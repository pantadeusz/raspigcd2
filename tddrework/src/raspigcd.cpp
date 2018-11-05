#include <configuration.hpp>
#include <hardware/motor_layout.hpp>
#include <hardware/raspberry_pi.hpp>
#include <hardware/stepping.hpp>
#include <movement/constant_speed.hpp>

using namespace raspigcd;
using namespace raspigcd::hardware;

//constant_speed

int main() {
	using namespace std::chrono_literals;

	configuration::global cfg;
	cfg.load_defaults();
	std::shared_ptr<raspberry_pi_3> raspi3( new raspberry_pi_3( cfg ) );
	std::shared_ptr<motor_layout> motor_layout_ = motor_layout::get_instance( cfg );
	movement::constant_speed const_speed_driver( motor_layout_ );
	stepping_simple_timer stepping( cfg, raspi3 );

	raspi3.get()->enable_steppers( {true} );

	auto commands = const_speed_driver.goto_xyz( {0, 0, 0, 0}, {0, 0, 2, 0}, 30, cfg.tick_duration() );
	stepping.exec( {0, 0, 0, 0}, commands, []( const steps_t& ) {} );
	commands = const_speed_driver.goto_xyz( {0, 0, 2, 0}, {0, 0, 0, 0}, 30, cfg.tick_duration() );
	stepping.exec( {0, 0, 0, 0}, commands, []( const steps_t& ) {} );


	raspi3.get()->enable_steppers( {false} );
	return 0;
}

int main_old() {
	using namespace std::chrono_literals;

	configuration::global cfg;
	cfg.load_defaults();
	raspberry_pi_3 raspi3( cfg );
	raspi3.enable_steppers( {true} );
	for ( int i = 0; i < cfg.steppers[2].steps_per_mm * 2; i++ ) {
		single_step_command cmnd[4];
		cmnd[3].step = 0;
		cmnd[0] = cmnd[1] = cmnd[2] = cmnd[3];
		cmnd[2].step = 1;
		cmnd[2].dir = 1;
		raspi3.do_step( cmnd );
		std::this_thread::sleep_for( 1ms );
	}
	for ( int i = 0; i < cfg.steppers[2].steps_per_mm * 2; i++ ) {
		single_step_command cmnd[4];
		cmnd[3].step = 0;
		cmnd[0] = cmnd[1] = cmnd[2] = cmnd[3];
		cmnd[2].step = 1;
		cmnd[2].dir = 0;
		raspi3.do_step( cmnd );
		std::this_thread::sleep_for( 6ms );
	}
	raspi3.enable_steppers( {false} );
	return 0;
}