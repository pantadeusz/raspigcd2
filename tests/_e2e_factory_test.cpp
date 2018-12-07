#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>
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
#include <hardware/driver/low_timers_busy_wait.hpp>
#include <gcd/factory.hpp>

using namespace raspigcd;
using namespace raspigcd::hardware;


TEST_CASE("gcd::path_intent_executor_factory in memory execute whole program", "[gcd][path_intent_executor_factory][e2e]")
{
    using namespace std::chrono_literals;

    configuration::global cfg;
    cfg.load_defaults();
    std::shared_ptr<raspigcd::gcd::path_intent_executor> executor_p = gcd::path_intent_executor_factory(cfg, gcd::machine_driver_selection::IN_MEMORY);
    auto &executor =*(executor_p.get());
    REQUIRE_NOTHROW( executor.execute(
        {
            movement::path_intentions::motor_t{.delay_s = 0.5, .motor = {true,true,true,true}},
            movement::path_intentions::spindle_t{.delay_s = 0.0001, .spindle = {{0, 1.0}}},
            distance_t{0,0,0,0},
            movement::path_intentions::move_t(20.0),
            distance_t{20,0,20,0},
            movement::path_intentions::spindle_t{.delay_s = 0.01, .spindle = {{0, 0.0}}},
            distance_t{20,0,20,0},
            movement::path_intentions::move_t(20.0),
            distance_t{0,0,0,0},
            movement::path_intentions::motor_t{.delay_s = 0.001, .motor = {false,false,false,false}},
        })
    );
}

/*

int main_spindle_test()
{
    using namespace std::chrono_literals;

    configuration::global cfg;
    cfg.load_defaults();
    std::shared_ptr<driver::raspberry_pi_3> raspi3 = std::make_shared<driver::raspberry_pi_3>(cfg);

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
    std::shared_ptr<driver::raspberry_pi_3> raspi3 = std::make_shared<driver::raspberry_pi_3>(cfg);
    std::shared_ptr<motor_layout> motor_layout_ = motor_layout::get_instance(cfg);
;
    movement::steps_generator steps_generator_drv(motor_layout_);
    stepping_simple_timer stepping(cfg, raspi3, std::make_shared<hardware::driver::low_timers_busy_wait>());

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

*/