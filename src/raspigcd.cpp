#include <configuration.hpp>
#include <hardware/motor_layout.hpp>
#include <hardware/driver/raspberry_pi.hpp>
#include <hardware/stepping.hpp>
#include <movement/steps_generator.hpp>
#include <movement/variable_speed.hpp>

using namespace raspigcd;
using namespace raspigcd::hardware;


int main()
{
    using namespace std::chrono_literals;

    configuration::global cfg;
    cfg.load_defaults();
//    std::shared_ptr<driver::raspberry_pi_3> raspi3(new driver::raspberry_pi_3(cfg));
    std::shared_ptr<driver::raspberry_pi_3> raspi3(new driver::raspberry_pi_3(cfg));
    std::shared_ptr<motor_layout> motor_layout_ = motor_layout::get_instance(cfg);
    movement::steps_generator steps_generator_drv(motor_layout_);
    stepping_simple_timer stepping(cfg, raspi3);



	movement::variable_speed variable_speed_driver( motor_layout_, cfg.max_no_accel_velocity_mm_s[0], cfg.max_accelerations_mm_s2[0], cfg.max_velocity_mm_s[0], cfg.tick_duration() );

    raspi3.get()->enable_steppers({true});

    
    std::list<std::variant<distance_t, double>> simple_program = {
			distance_t{0, 0, 0, 0}, 15.0,
			distance_t{0, 0, 2, 0}, 5.0,
			distance_t{0, 0, 0, 0}
		};
	auto plan_to_execute = variable_speed_driver.intent_to_movement_plan( simple_program );

    steps_t psteps = {0, 0, 0, 0};
    std::list < std::unique_ptr<std::vector<hardware::multistep_command> > > ticks_to_execute;
    steps_generator_drv.movement_plan_to_step_commands(plan_to_execute, cfg.tick_duration(), 
        [&psteps,&stepping,&ticks_to_execute](std::unique_ptr<std::vector<hardware::multistep_command> > msteps_p){
            ticks_to_execute.push_back(std::move(msteps_p));
        });
    for (auto & msteps_p : ticks_to_execute) {
        std::cout << "should execute: " << msteps_p.get()->size() << std::endl;
        //psteps = stepping.exec(psteps, *(msteps_p.get()), [](const steps_t&) {});      
    }    
    raspi3.get()->enable_steppers({false});
    return 0;
}


int main_spindle_test()
{
    using namespace std::chrono_literals;

    configuration::global cfg;
    cfg.load_defaults();
    std::shared_ptr<driver::raspberry_pi_3> raspi3(new driver::raspberry_pi_3(cfg));

    raspi3.get()->enable_steppers({true});

    std::this_thread::sleep_for(3s);
    raspi3.get()->spindle_pwm_power(0, 1);
    std::this_thread::sleep_for(3s);
    raspi3.get()->spindle_pwm_power(0, 0);

    raspi3.get()->enable_steppers({false});
    return 0;
}


int main_old2()
{
    using namespace std::chrono_literals;

    configuration::global cfg;
    cfg.load_defaults();
    std::shared_ptr<driver::raspberry_pi_3> raspi3(new driver::raspberry_pi_3(cfg));
    std::shared_ptr<motor_layout> motor_layout_ = motor_layout::get_instance(cfg);
    movement::steps_generator steps_generator_drv(motor_layout_);
    stepping_simple_timer stepping(cfg, raspi3);

    raspi3.get()->enable_steppers({true});

    auto commands = steps_generator_drv.goto_xyz({0, 0, 0, 0}, {0, 0, 2, 0}, 30, cfg.tick_duration());
    stepping.exec({0, 0, 0, 0}, commands, [](const steps_t&) {});
    commands = steps_generator_drv.goto_xyz({0, 0, 2, 0}, {0, 0, 0, 0}, 30, cfg.tick_duration());
    stepping.exec({0, 0, 0, 0}, commands, [](const steps_t&) {});


    raspi3.get()->enable_steppers({false});
    return 0;
}

int main_old()
{
    using namespace std::chrono_literals;

    configuration::global cfg;
    cfg.load_defaults();
    driver::raspberry_pi_3 raspi3(cfg);
    raspi3.enable_steppers({true});
    for (int i = 0; i < cfg.steppers[2].steps_per_mm * 2; i++) {
        single_step_command cmnd[4];
        cmnd[3].step = 0;
        cmnd[0] = cmnd[1] = cmnd[2] = cmnd[3];
        cmnd[2].step = 1;
        cmnd[2].dir = 1;
        raspi3.do_step(cmnd);
        std::this_thread::sleep_for(1ms);
    }
    for (int i = 0; i < cfg.steppers[2].steps_per_mm * 2; i++) {
        single_step_command cmnd[4];
        cmnd[3].step = 0;
        cmnd[0] = cmnd[1] = cmnd[2] = cmnd[3];
        cmnd[2].step = 1;
        cmnd[2].dir = 0;
        raspi3.do_step(cmnd);
        std::this_thread::sleep_for(6ms);
    }
    raspi3.enable_steppers({false});
    return 0;
}