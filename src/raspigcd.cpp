#include <configuration.hpp>
#include <hardware/motor_layout.hpp>
#include <hardware/driver/raspberry_pi.hpp>
#include <hardware/driver/inmem.hpp>
#include <hardware/stepping.hpp>
#include <movement/steps_generator.hpp>
#include <movement/variable_speed.hpp>
#include <gcd/path_intent_executor.hpp>
#include <hardware/driver/low_buttons_fake.hpp>
#include <hardware/driver/low_spindles_pwm_fake.hpp>
#include <hardware/driver/low_timers_fake.hpp>

using namespace raspigcd;
using namespace raspigcd::hardware;


int main()
{
    using namespace std::chrono_literals;

    configuration::global cfg;
    cfg.load_defaults();

    raspigcd::gcd::gcode_interpreter_objects_t objs{};
    objs.motor_layout = hardware::motor_layout::get_instance(cfg);
    objs.configuration.motion_layout = cfg.motion_layout;
    objs.configuration.scale = cfg.scale;
    objs.configuration.steppers = cfg.steppers;
    objs.configuration = cfg;
    objs.stepping = std::make_shared<hardware::stepping_simple_timer>(objs.configuration, objs.steppers);


    try {
        auto hardware_driver = std::make_shared< driver::raspberry_pi_3>(cfg);
        objs.steppers = hardware_driver;
        objs.buttons = hardware_driver;
        objs.spindles_pwm = hardware_driver;
        objs.timers = hardware_driver;
    } catch ( ... ) {
        std::cout << "falling back to emulation of hardware motors..." << std::endl;
        objs.steppers = std::make_shared< driver::inmem>();
        objs.buttons = std::make_shared<hardware::driver::low_buttons_fake>();
        objs.spindles_pwm = std::make_shared<hardware::driver::low_spindles_pwm_fake>();
        objs.timers = std::make_shared<hardware::driver::low_timers_fake>();
    }
    objs.stepping = std::make_shared<hardware::stepping_simple_timer>(objs.configuration, objs.steppers);
    
    raspigcd::gcd::path_intent_executor executor;
    executor.set_gcode_interpreter_objects(objs);
    auto result = executor.execute(
        {
            //movement::path_intentions::spindle_t{.delay_s = 0.01, .spindle = {{0, 1.0}}},
            movement::path_intentions::motor_t{.delay_s = 0.001, .motor = {true,true,true,true}},
            distance_t{0,0,0,0},
            movement::path_intentions::move_t(10.0),
            distance_t{1,2,3,0},
            movement::path_intentions::pause_t{.delay_s=0.0},
            distance_t{0,0,0,0},
            movement::path_intentions::move_t(10.0),
            distance_t{-1,-2,-3,0},
            //movement::path_intentions::spindle_t{.delay_s = 0.01, .spindle = {{0, 0.0}}},
            movement::path_intentions::motor_t{.delay_s = 0.001, .motor = {false,false,false,false}},
        });


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

    auto commands = steps_generator_drv.movement_from_to({0, 0, 0, 0}, {.v0 = 30,.accel = 0,.max_v = 30}, {0, 0, 2, 0}, cfg.tick_duration());
    stepping.exec(commands);
    commands = steps_generator_drv.movement_from_to({0, 0, 2, 0}, {.v0 = 30,.accel = 0,.max_v = 30}, {0, 0, 0, 0}, cfg.tick_duration());
    stepping.exec(commands);


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