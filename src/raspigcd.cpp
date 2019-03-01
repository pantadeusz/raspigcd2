/*
    Raspberry Pi G-CODE interpreter

    Copyright (C) 2019  Tadeusz Puźniakowski puzniakowski.pl

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/


/*

This is simple program that uses the library. It will execute given GCode.

*/

#include <configuration.hpp>
#include <converters/gcd_program_to_steps.hpp>
#include <hardware/driver/inmem.hpp>
#include <hardware/driver/low_buttons_fake.hpp>
#include <hardware/driver/low_spindles_pwm_fake.hpp>
#include <hardware/driver/low_timers_busy_wait.hpp>
#include <hardware/driver/low_timers_wait_for.hpp>
#include <hardware/driver/low_timers_fake.hpp>
#include <hardware/driver/raspberry_pi.hpp>
#include <hardware/motor_layout.hpp>
#include <hardware/stepping.hpp>


#include <fstream>
#include <streambuf>
#include <string>
#include <mutex>

using namespace raspigcd;
using namespace raspigcd::hardware;
using namespace raspigcd::gcd;

/// Visualization part

#ifdef HAVE_SDL2

#include <SDL2/SDL.h>
class video_sdl {
public:
    std::shared_ptr<SDL_Window> window;
    std::shared_ptr<SDL_Renderer> renderer;

    std::atomic<bool> active;

    std::thread loop_thread;

    steps_t current_steps;

    std::list<steps_t> movements_track;
    steps_t steps_scale;

    std::mutex list_mutex;
    configuration::global *cfg;

    void set_steps(const steps_t&st) {
        std::lock_guard<std::mutex> guard(list_mutex);
        current_steps = st/steps_scale;
        if (!(movements_track.back() == current_steps)) {
            movements_track.push_back(current_steps);    
        }

    }

    video_sdl(configuration::global *cfg_, int width = 640, int height = 480) {
        cfg = cfg_;
        
        steps_scale = {cfg->steppers[0].steps_per_mm,cfg->steppers[1].steps_per_mm,cfg->steppers[2].steps_per_mm,cfg->steppers[3].steps_per_mm};
        movements_track.push_back({0,0,0,0});
        loop_thread = std::thread([this,width,height](){
            std::cout << "loop thread..." << std::endl;

            SDL_Window *win = SDL_CreateWindow( "Witaj w Swiecie",
                                                SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                                width, height, SDL_WINDOW_SHOWN );
            if ( win == nullptr ) throw std::invalid_argument ( "SDL_CreateWindow - error" );
            window = std::shared_ptr<SDL_Window> ( win, []( SDL_Window * ptr ) {
                SDL_DestroyWindow( ptr );
            } );
            
            SDL_Renderer *ren = SDL_CreateRenderer( window.get(), -1, SDL_RENDERER_ACCELERATED );
            if ( ren == nullptr ) throw std::invalid_argument ( "SDL_CreateRenderer" );
            renderer = std::shared_ptr<SDL_Renderer> ( ren, []( SDL_Renderer * ptr ) {
                SDL_DestroyRenderer( ptr );
            } );
            for ( ; active; ) {
                    SDL_Event event;
                    while ( SDL_PollEvent( &event ) ) {
                        if ( event.type == SDL_QUIT ) {
                            active = false;
                            std::cout << "window closed" << std::endl;
                        }
                    }
                    
                    SDL_SetRenderDrawColor(renderer.get(),0,0,0,255);
                    SDL_RenderClear( renderer.get() );
                    
                    SDL_SetRenderDrawColor(renderer.get(),255,255,255,255);
                    steps_t s;
                    std::list<steps_t> t;

                    {
                    std::lock_guard<std::mutex> guard(list_mutex);
                    s = current_steps;
                    t = movements_track;
                    }
                    for (auto e : t) {
                        SDL_RenderDrawPoint(renderer.get(), e[0]+width/2, e[1]+height/2);    
                    }
                    SDL_RenderDrawPoint(renderer.get(), s[0]+width/2-s[2], s[1]+s[2]+height/2);
                    //std::cout << "step.. " << s[0] << ", " << s[1] << ", " << s[2] << std::endl;

                    SDL_RenderPresent( renderer.get() );
                    SDL_Delay( 33 );
                    
            }
        });

    }
    virtual ~video_sdl(){
        //active = false;
        loop_thread.join();
    }
};

#else
class video_sdl {
public:
    void set_steps(const steps_t&st) {
    }

    video_sdl(configuration::global *cfg_, int width = 640, int height = 480) {
    }
    virtual ~video_sdl(){
    }
};
#endif
/// end of visualization



int main(int argc, char** argv)
{
    #ifdef HAVE_SDL2
    //std::cout << "WITH SDL!!! " << std::endl;
    if ( SDL_Init( SDL_INIT_VIDEO ) != 0 ) throw std::invalid_argument ( "SDL_Init" );
    #endif

    using namespace std::chrono_literals;
    std::vector<std::string> args(argv, argv + argc);
    configuration::global cfg;
    cfg.load_defaults();

    bool raw_gcode = false; // should I push G commands directly, without adaptation to machine

    for (unsigned i = 1; i < args.size(); i++) {
        if ((args.at(i) == "-h") || (args.at(i) == "--help")) {
            std::cout << "NAME" << std::endl;
            std::cout << "\t" << args[0] << " - raspigcd runner program." << std::endl;
            std::cout << std::endl;
            std::cout << "SYNOPSIS" << std::endl;
            std::cout << "\t" << args[0] << " [options]" << std::endl;
            std::cout << std::endl;
            std::cout << "DESCRIPTION" << std::endl;
            std::cout << "\tIt allows for execution of gcode on Raspberry Pi and simulation of such on desktop." << std::endl;
            std::cout << std::endl;
            std::cout << "\t-c <configfile>" << std::endl;
            std::cout << "\t\tprovide configuration file json" << std::endl;
            std::cout << std::endl;
            std::cout << "\t-f <filename>" << std::endl;
            std::cout << "\t\tgcode file to execute" << std::endl;
            std::cout << std::endl;
            std::cout << "\t-h" << std::endl;
            std::cout << "\t\thelp screen" << std::endl;
            std::cout << std::endl;
            std::cout << "\t--raw" << std::endl;
            std::cout << "\t\tTreat the file as raw - no additional processing. No limits check." << std::endl;
            std::cout << std::endl;
            std::cout << "AUTHOR" << std::endl;
            std::cout << "\tTadeusz Puźniakowski" << std::endl;
            std::cout << std::endl;
            std::cout << "REPORTING BUGS" << std::endl;
            std::cout << "\tOnline at <https://github.com/pantadeusz/raspigcd2>" << std::endl;
            std::cout << std::endl;
            std::cout << "COPYRIGHT" << std::endl;
            std::cout << "\t" << std::endl;
            std::cout << "\tRaspberry Pi G-CODE interpreter" << std::endl;
            std::cout << "\t" << std::endl;
            std::cout << "\tCopyright (C) 2019  Tadeusz Puźniakowski puzniakowski.pl" << std::endl;
            std::cout << "\t" << std::endl;
            std::cout << "\tThis program is free software: you can redistribute it and/or modify" << std::endl;
            std::cout << "\tit under the terms of the GNU Affero General Public License as published by" << std::endl;
            std::cout << "\tthe Free Software Foundation, either version 3 of the License, or" << std::endl;
            std::cout << "\t(at your option) any later version." << std::endl;
            std::cout << "\t" << std::endl;
            std::cout << "\tThis program is distributed in the hope that it will be useful," << std::endl;
            std::cout << "\tbut WITHOUT ANY WARRANTY; without even the implied warranty of" << std::endl;
            std::cout << "\tMERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the" << std::endl;
            std::cout << "\tGNU Affero General Public License for more details." << std::endl;
            std::cout << "\t" << std::endl;
            std::cout << "\tYou should have received a copy of the GNU Affero General Public License" << std::endl;
            std::cout << "\talong with this program.  If not, see <https://www.gnu.org/licenses/>." << std::endl;

        } else if (args.at(i) == "-c") {
            i++;
            cfg.load(args.at(i));
        } else if (args.at(i) == "--raw") {
            raw_gcode = true;
        } else if (args.at(i) == "-f") {
            using namespace raspigcd;
            using namespace raspigcd::hardware;

            std::shared_ptr<low_steppers> raspi3;
            std::shared_ptr<low_spindles_pwm> spindles;
            std::shared_ptr<video_sdl> video;
            steps_t position_for_fake;
            try {
                auto rp = std::make_shared<driver::raspberry_pi_3>(cfg);
                raspi3 = rp;
                spindles = rp;
            } catch (...) {
                video = std::make_shared<video_sdl>(&cfg);
                auto fk = std::make_shared<driver::inmem>();
                fk->set_step_callback([&position_for_fake, &video](const steps_t&st){
                    //if (!(position_for_fake == st)) {
                    //    std::cout << ";; " 
                    //    << st[0] << " " 
                    //    << st[1] << " " 
                    //    << st[2] << std::endl;
                    //}
                    if (!(position_for_fake == st)) {
                        if (video.get() != nullptr) video->set_steps(st);
                    }
                    position_for_fake = st;
                    //
                });
                raspi3 = fk;
                spindles = std::make_shared<raspigcd::hardware::driver::low_spindles_pwm_fake>(
                    [](const int s_i, const double p_i){
                        std::cout << "SPINDLE " << s_i << " POWER: " << p_i << std::endl;
                    }
                );
            }
            //std::shared_ptr<driver::raspberry_pi_3> raspi3 = std::make_shared<driver::raspberry_pi_3>(cfg);
            std::shared_ptr<motor_layout> motor_layout_ = motor_layout::get_instance(cfg);
            motor_layout_->set_configuration(cfg);

            std::shared_ptr<raspigcd::hardware::low_timers>timer_drv;
            switch (cfg.lowleveltimer) {
                case raspigcd::configuration::low_timers_e::WAIT_FOR: timer_drv = std::make_shared<hardware::driver::low_timers_busy_wait>(); break;
                case raspigcd::configuration::low_timers_e::BUSY_WAIT: timer_drv = std::make_shared<hardware::driver::low_timers_wait_for>(); break;
                case raspigcd::configuration::low_timers_e::FAKE: timer_drv = std::make_shared<hardware::driver::low_timers_fake>(); break;
            }
            stepping_simple_timer stepping(cfg, raspi3, timer_drv);

            i++;
            std::ifstream gcd_file(args.at(i));
            if (!gcd_file.is_open()) throw std::invalid_argument("file should be opened");
            std::string gcode_text((std::istreambuf_iterator<char>(gcd_file)),
                std::istreambuf_iterator<char>());

            auto program = gcode_to_maps_of_arguments(gcode_text);
            for (auto &p: program) {
                if ((p.count('G')) && (p['G'] == 0)) 
                    p['F'] = *std::max_element(std::begin(cfg.max_velocity_mm_s), std::end(cfg.max_velocity_mm_s));
            }
            auto program_parts = group_gcode_commands(program);
            std::cout << "Initial GCODE: " << std::endl << back_to_gcode(program_parts) << std::endl;
            block_t machine_state = {{'F', 0.5}};
            program_parts = insert_additional_nodes_inbetween(program_parts, machine_state, cfg);
            std::cout << "Initial GCODE w additional parts: " << std::endl << back_to_gcode(program_parts) << std::endl;
            
            program_t prepared_program;
            if (!raw_gcode) {
            for (auto& ppart : program_parts) {
                if (ppart.size() != 0) {
                    if (ppart[0].count('M') == 0) {
                        std::cout << "G PART: " << ppart.size() << std::endl;
                        switch ((int)(ppart[0]['G'])) {
                        case 4:
                            std::cout << "Dwell not supported" << std::endl;
                            break;
                        case 0:
                            //ppart = g0_move_to_g1_sequence(ppart, cfg, machine_state);
                            //for (auto &e : ppart) e['G'] = 0;
                            //prepared_program.insert(prepared_program.end(), ppart.begin(), ppart.end());
                            //machine_state = last_state_after_program_execution(ppart,machine_state);
                            //break;
                        case 1:
                            ppart = g1_move_to_g1_with_machine_limits(ppart, cfg, machine_state);
                            prepared_program.insert(prepared_program.end(), ppart.begin(), ppart.end());
                            machine_state = last_state_after_program_execution(ppart,machine_state);
                            break;
                        }
                    } else {
                        std::cout << "M PART: " << ppart.size() << std::endl;
                        for (auto& m : ppart) {
                            switch ((int)(m['M'])) {
                                case 18:
                                case 3:
                                case 5:
                                case 17:
                                prepared_program.push_back(m);
                                break;
                            }
                        }
                    }
                }
            }


            std::cout << "Prepared GCODE INIT: " << std::endl << back_to_gcode(group_gcode_commands(prepared_program)) << std::endl;
            std::cout << "Prepared GCODE NO DUPLICATES: " << std::endl << back_to_gcode({remove_duplicate_blocks(prepared_program,{})}) << std::endl;
            program_parts = group_gcode_commands(remove_duplicate_blocks(prepared_program,{}));
            //program_parts = group_gcode_commands(prepared_program);
            std::cout << "Prepared GCODE: " << std::endl << back_to_gcode(program_parts) << std::endl;
            machine_state = {{'F', 0.5}};
            }
            std::cout << "STARTING" << std::endl;
            for (auto& ppart : program_parts) {
                if (ppart.size() != 0) {
                    if (ppart[0].count('M') == 0) {
                        std::cout << "G PART: " << ppart.size() << std::endl;
                        switch ((int)(ppart[0]['G'])) {
                        case 4:
                            std::cout << "Dwell not supported" << std::endl;
                            break;
                        case 0:
                        case 1:
                            auto m_commands = converters::program_to_steps(ppart, cfg, *(motor_layout_.get()),
                                machine_state, [&machine_state](const block_t& result) {
                                    machine_state = result;
                                    for (auto& s : machine_state) {
                                        std::cout << s.first << ":" << s.second << " ";
                                    }
                                    std::cout << std::endl;
                                });
                            stepping.exec(m_commands);
                            std::list<steps_t> steps = hardware_commands_to_steps(m_commands);
                            std::cout << "steps: " << motor_layout_->steps_to_cartesian(steps.back()) << std::endl;
                            break;
                        }
                    } else {
                        std::cout << "M PART: " << ppart.size() << std::endl;
                        for (auto& m : ppart) {
                            switch ((int)(m['M'])) {
                            case 17:
                                raspi3->enable_steppers({true});
                                break;
                            case 18:
                                raspi3->enable_steppers({false});
                                break;
                            case 3:
                                spindles->spindle_pwm_power(0, 1.0);
                                break;
                            case 5:
                                spindles->spindle_pwm_power(0, 0.0);
                                break;
                            }
                        }
                    }
                }
                std::cout << "[MS] Machine state" << std::endl;
                for (auto& s : machine_state) {
                    std::cout << s.first << ":" << s.second << " ";
                }
                std::cout << "  OK" << std::endl;
            }

        }
    }
    #ifdef HAVE_SDL2
    SDL_Quit();
    #endif
    /*raspigcd::gcd::gcode_interpreter_objects_t objs{};
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
    */
    /*
    std::shared_ptr<raspigcd::gcd::path_intent_executor> executor_p = gcd::path_intent_executor_factory(cfg, gcd::machine_driver_selection::RASPBERRY_PI);
    auto& executor = *(executor_p.get());
    auto result = executor.execute(
        {
            movement::path_intentions::motor_t{.delay_s = 0.5, .motor = {true, true, true, true}},
            movement::path_intentions::spindle_t{.delay_s = 0.0001, .spindle = {{0, 1.0}}},
            distance_t{0, 0, 0, 0},
            movement::path_intentions::move_t(20.0),
            distance_t{20, 0, 20, 0},
            movement::path_intentions::spindle_t{.delay_s = 0.01, .spindle = {{0, 0.0}}},
            distance_t{20, 0, 20, 0},
            movement::path_intentions::move_t(20.0),
            distance_t{0, 0, 0, 0},
            movement::path_intentions::motor_t{.delay_s = 0.001, .motor = {false, false, false, false}},
        });
*/

    return 0;
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

    auto commands = steps_generator_drv.movement_from_to({0, 0, 0, 0}, {.v0 = 30, .accel = 0, .max_v = 30}, {0, 0, 2, 0}, cfg.tick_duration());
    stepping.exec(commands);
    commands = steps_generator_drv.movement_from_to({0, 0, 2, 0}, {.v0 = 30, .accel = 0, .max_v = 30}, {0, 0, 0, 0}, cfg.tick_duration());
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
