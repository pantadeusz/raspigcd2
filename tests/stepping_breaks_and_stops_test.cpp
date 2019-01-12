#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>
#include <chrono>
#include <condition_variable>
#include <hardware/driver/inmem.hpp>
#include <hardware/driver/low_buttons_fake.hpp>
#include <hardware/driver/low_spindles_pwm_fake.hpp>
#include <hardware/driver/low_timers_fake.hpp>
#include <hardware/stepping.hpp>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

/*
Terminate the execution of steps program
*/


using namespace raspigcd;
using namespace raspigcd::hardware;
//using namespace raspigcd::movement;
using namespace raspigcd::configuration;


// void path_intent_executor::execute_pure_path_intent(const movement::path_intent_t& path_intent) {
TEST_CASE("path_intent_executor terminate procedure for stepping_simple_timer.", "[gcd][path_intent_executor][stepping_simple_timer][execute_pure_path_intent]")
{
    auto steppers_drv = std::make_shared<hardware::driver::inmem>();
    auto timers_drv = std::make_shared<hardware::driver::low_timers_fake>();

    std::shared_ptr<hardware::stepping> stepping = std::make_shared<hardware::stepping_simple_timer>(50, steppers_drv, timers_drv);


    hardware::driver::inmem* inmem_ptr = (hardware::driver::inmem*)steppers_drv.get();
    //hardware::driver::low_timers_fake* low_timers_fake_ptr = (hardware::driver::low_timers_fake*)timers_drv.get();

    SECTION("break execution and check that the steps performed are less than requested")
    {
        multistep_commands_t commands_to_do;
        for (int i = 0; i < 4; i++) {
            //int n = 0;
            multistep_command cmnd;
            cmnd.count = 1;
            cmnd.b[0].step = 0;
            cmnd.b[0].dir = 0;
            cmnd.b[1].step = 0;
            cmnd.b[1].dir = 0;
            cmnd.b[2].step = 0;
            cmnd.b[2].dir = 0;
            cmnd.b[3].step = 0;
            cmnd.b[3].dir = 0;
            cmnd.b[i].step = 1;
            cmnd.b[i].dir = 0;
            commands_to_do.push_back(cmnd);
        }

        std::mutex m;
        std::mutex n;

        int i = 0;
        inmem_ptr->set_step_callback([&](const steps_t&) {
            if (i == 1) { m.unlock(); n.lock(); } // sync point after first step
            i++;
        });
        m.lock();
        n.lock();
        std::thread worker([&]() {
            try {
                stepping.get()->exec(commands_to_do);
            } catch( const raspigcd::hardware::execution_terminated & e) {
            }
        });
        m.lock();
        stepping.get()->terminate();
        n.unlock();

        worker.join();
        REQUIRE(stepping.get()->get_tick_index() == 2);
        REQUIRE(stepping.get()->get_tick_index() == i);

        std::list<steps_t> steps_from_commands_ = hardware_commands_to_steps(commands_to_do);
        std::vector<steps_t> steps_from_commands(steps_from_commands_.begin(),steps_from_commands_.end());
        REQUIRE(inmem_ptr->current_steps == steps_from_commands[stepping.get()->get_tick_index()-1]);
    }
    /*
    
    SECTION("execute not that simple program and check result")
    {
        int steps_counter = 0;
        std::mutex set_step_callback_mutex;
        std::mutex set_step_callback_mutex_2;
        inmem_ptr->set_step_callback([&](const steps_t&) {
            if (steps_counter == 100) {
                set_step_callback_mutex_2.unlock();
                set_step_callback_mutex.lock(); // here it will wait until lock is freed
            }
        });

        set_step_callback_mutex.lock();
        set_step_callback_mutex_2.lock();
        executor.set_gcode_interpreter_objects(objs);
        // it should stop after 100 steps
        auto handle = std::async(std::launch::async,[&](){
            executor.execute({
                distance_t{0,0,0,0},
                movement::path_intentions::move_t(48.0),
                distance_t{1,2,3,4},
                movement::path_intentions::move_t(20.0),
                distance_t{2,1,0,2}
            });
        });
        set_step_callback_mutex_2.lock(); // wait for thread to unlock
        // do the break!
        executor.break();
        set_step_callback_mutex.unlock(); // now we unlock
//        executor.get
//        REQUIRE(inmem_ptr->current_steps == objs.motor_layout.get()->cartesian_to_steps(distance_t{2,1,0,2}));
    }
    */
}


TEST_CASE("path_intent_executor terminate procedure on stepping_sim for verification.", "[gcd][path_intent_executor][stepping_sim][execute_pure_path_intent]")
{
    SECTION("break execution and check that the steps performed are less than requested")
    {
        multistep_commands_t commands_to_do;
        for (int i = 0; i < 4; i++) {
            multistep_command cmnd;
            cmnd.count = 1;
            cmnd.b[0].step = 0;
            cmnd.b[0].dir = 0;
            cmnd.b[1].step = 0;
            cmnd.b[1].dir = 0;
            cmnd.b[2].step = 0;
            cmnd.b[2].dir = 0;
            cmnd.b[3].step = 0;
            cmnd.b[3].dir = 0;
            cmnd.b[i].step = 1;
            cmnd.b[i].dir = 0;
            commands_to_do.push_back(cmnd);
        }

        std::mutex m;
        std::mutex n;

        int i = 0;

        steps_t current_steps;
        std::shared_ptr<hardware::stepping> stepping = std::make_shared<raspigcd::hardware::stepping_sim>(
            steps_t{0,0,0,0},
            [&](const steps_t&s) {
                if (i == 1) {
                    m.unlock();
                    n.lock();
                }
                current_steps = s;
                i++;
            }
            );

        m.lock();
        n.lock();
        std::thread worker([&]() {
            try {
                stepping.get()->exec(commands_to_do);
            } catch( const raspigcd::hardware::execution_terminated & e) {
            }
        });
        //worker.detach();
        m.lock();
        stepping.get()->terminate();
        n.unlock();

        worker.join();
        REQUIRE(stepping.get()->get_tick_index() == 2);
        REQUIRE(stepping.get()->get_tick_index() == i);
        std::list<steps_t> steps_from_commands_ = hardware_commands_to_steps(commands_to_do);
        std::vector<steps_t> steps_from_commands(steps_from_commands_.begin(),steps_from_commands_.end());
        REQUIRE(current_steps == steps_from_commands[stepping.get()->get_tick_index()-1]);
    }
}

