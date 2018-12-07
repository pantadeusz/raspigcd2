#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>
#include <chrono>
#include <gcd/path_intent_executor.hpp>
#include <hardware/driver/inmem.hpp>
#include <hardware/driver/low_buttons_fake.hpp>
#include <hardware/driver/low_spindles_pwm_fake.hpp>
#include <hardware/driver/low_timers_fake.hpp>
#include <hardware/stepping.hpp>
#include <thread>
#include <vector>

/*
This module is supposed to let me test if the breaking methods works as expected. We need to have certainity that the following methods works ok:

terminate - it should stop everything at once: movement, spindles and motors. This should allow for retrieving end position.
pause - perform break procedure and disable spindle. Leave stepper motors on. Wait for resume command
stop - first pause, then exit program. This should allow for getting the current coorinates, so the GCODE interpreter can work properly
resume - continue paused work

*/


using namespace raspigcd;
using namespace raspigcd::movement;
using namespace raspigcd::configuration;

// void path_intent_executor::execute_pure_path_intent(const movement::path_intent_t& path_intent) {
TEST_CASE("path_intent_executor terminate procedure.", "[gcd][path_intent_executor][execute_pure_path_intent]")
{
    configuration::global actuators_cfg{};
    actuators_cfg.motion_layout = CARTESIAN; 
    actuators_cfg.scale = {1.0, 1.0, 1.0, 1.0};
    actuators_cfg.steppers = {
        stepper(27, 10, 22, 100.0),
        stepper(4, 10, 17, 100.0),
        stepper(9, 10, 11, 100.0),
        stepper(0, 10, 5, 100.0)};
    actuators_cfg.tick_duration_us = 50;

    actuators_cfg.max_accelerations_mm_s2 = {2000.0, 2000.0, 2000.0, 2000.0};
    actuators_cfg.max_velocity_mm_s = {220.0, 220.0, 110.0, 220.0};  ///<maximal velocity on axis in mm/s
    actuators_cfg.max_no_accel_velocity_mm_s = {2.0, 2.0, 2.0, 2.0}; ///<maximal velocity on axis in mm/s


    raspigcd::gcd::path_intent_executor executor;
    
    raspigcd::gcd::gcode_interpreter_objects_t objs{};
    objs.buttons = std::make_shared<hardware::driver::low_buttons_fake>();
    objs.steppers = std::make_shared<hardware::driver::inmem>();
    objs.spindles_pwm = std::make_shared<hardware::driver::low_spindles_pwm_fake>();
    objs.timers = std::make_shared<hardware::driver::low_timers_fake>();
    objs.motor_layout = hardware::motor_layout::get_instance(actuators_cfg);
    objs.configuration.motion_layout = actuators_cfg.motion_layout;
    objs.configuration.scale = actuators_cfg.scale;
    objs.configuration.steppers = actuators_cfg.steppers;
    objs.configuration = actuators_cfg;
    
    objs.stepping = std::make_shared<hardware::stepping_simple_timer>(objs.configuration, objs.steppers,objs.timers);


    hardware::driver::inmem* inmem_ptr = (hardware::driver::inmem*)objs.steppers.get();
    hardware::driver::low_timers_fake* low_timers_fake_ptr = (hardware::driver::low_timers_fake*)objs.timers.get();
    hardware::driver::low_spindles_pwm_fake* low_spindles_pwm_fake_ptr = (hardware::driver::low_spindles_pwm_fake*)objs.spindles_pwm.get();


    SECTION("execute and check if the end position is correct")
    {
        executor.set_gcode_interpreter_objects(objs);
        auto ret = executor.execute({
            distance_t{0,0,0,0},
            movement::path_intentions::move_t(48.0),
            distance_t{1,2,3,4},
            movement::path_intentions::move_t(20.0),
            distance_t{2,1,0,2}
        });
        REQUIRE(ret.position == objs.motor_layout.get()->cartesian_to_steps(distance_t{2,1,0,2}));
    }
    
    SECTION("execute adn terminate should throw exception")
    {
        int steps_counter = 0;
        std::mutex set_step_callback_mutex;
        std::mutex set_step_callback_mutex_2;
        inmem_ptr->set_step_callback([&](const steps_t&p) {
            if (steps_counter == 4000) {
                set_step_callback_mutex_2.unlock();
                set_step_callback_mutex.lock(); // here it will wait until lock is freed
            }
            steps_counter++;
        });

        set_step_callback_mutex.lock();
        set_step_callback_mutex_2.lock();
        executor.set_gcode_interpreter_objects(objs);

        auto handle = std::async(std::launch::async,[&](){
            auto result = executor.execute({
                distance_t{0,0,0,0},
                movement::path_intentions::move_t(48.0),
                distance_t{1,2,3,4},
                movement::path_intentions::move_t(20.0),
                distance_t{2,1,0,2}
            });
            return result;
        });
        set_step_callback_mutex_2.lock(); // wait for thread to unlock
        // do the break!
        executor.terminate();
        set_step_callback_mutex.unlock(); // now we unlock
        auto result = handle.get();
        REQUIRE(result.errors.size() > 0);
        REQUIRE(result.position == steps_t{130, 169, 207, 338});
    }

}
