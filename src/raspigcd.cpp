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
#include <hardware/driver/low_timers_fake.hpp>
#include <hardware/driver/low_timers_wait_for.hpp>
#include <hardware/driver/raspberry_pi.hpp>
#include <hardware/motor_layout.hpp>
#include <hardware/stepping.hpp>

#include <configuration_json.hpp>

#include <fstream>
#include <future>
#include <mutex>
#include <streambuf>
#include <string>

using namespace raspigcd;
using namespace raspigcd::hardware;
using namespace raspigcd::gcd;

/// Visualization part
//#define HAVE_SDL2

#ifdef HAVE_SDL2

#include <SDL2/SDL.h>
class video_sdl
{
public:
    std::shared_ptr<SDL_Window> window;
    std::shared_ptr<SDL_Renderer> renderer;

    std::atomic<bool> active;

    std::thread loop_thread;

    distance_t current_position;

    std::list<distance_t> movements_track;
    steps_t steps_scale;

    std::mutex list_mutex;
    configuration::global* cfg;
    std::shared_ptr<motor_layout> ml;


    int z_p_x; // 1000x
    int z_p_y; // 1000x
    int view_x;
    int view_y;


    void set_steps(const steps_t& st)
    {
        current_position = ml->steps_to_cartesian(st);
        steps_t reduced_a;
        steps_t reduced_b;

        std::lock_guard<std::mutex> guard(list_mutex);
        for (int i = 0; i < 4; i++) {
            reduced_a[i] = current_position[i];       ///(int)(cfg->steppers[i].steps_per_mm);
            reduced_b[i] = movements_track.back()[i]; ///(int)(cfg->steppers[i].steps_per_mm);
        }
        if (!(reduced_a == reduced_b)) {
            movements_track.push_back(current_position);
        }
    }

    void draw_path(std::shared_ptr<SDL_Renderer> renderer, int width, int /*height*/, const std::list<distance_t> &t) {
        std::map < int, int > z_buffer;
        
        for (auto &e : t) {
                    int x = e[0];
                    int y = e[1];
                    int z = e[2];
                    if (e[2] <= 0) {
                        if ((z_buffer.count (y*width+x) == 0) || (z_buffer[y*width+x] >= z) ) {
                            SDL_SetRenderDrawColor(renderer.get(), 255 - (e[2] * 255 / 5), 255, 255, 255);
                            SDL_RenderDrawPoint(renderer.get(), (x + view_x)+z*z_p_x/1000, (-y + view_y)-z*z_p_y/1000);
                            z_buffer[y*width+x] = z;
                        }
                    }
                }
    }

    video_sdl(configuration::global* cfg_, driver::low_buttons_fake* buttons_drv, int width = 640, int height = 480)
    {
        active = true;
        cfg = cfg_;

        z_p_x = 500; // 1000x
        z_p_y = 500; // 1000x
        view_x = width / 2;
        view_y = height / 2;

        ml = motor_layout::get_instance(*cfg);
        movements_track.push_back({0, 0, 0, 0});
        loop_thread = std::thread([this, width, height, buttons_drv]() {
            std::cout << "loop thread..." << std::endl;

            window = std::shared_ptr<SDL_Window>(SDL_CreateWindow("GCD Execution Simulator By Tadeusz Puzniakowski",
                SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                width, height, SDL_WINDOW_SHOWN), [](SDL_Window* ptr) {
                SDL_DestroyWindow(ptr);
            });
            if (window == nullptr) throw std::invalid_argument("SDL_CreateWindow - error");
            
            renderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(window.get(), -1, SDL_RENDERER_ACCELERATED), [](SDL_Renderer* ptr) {
                SDL_DestroyRenderer(ptr);
            });
            if (renderer == nullptr) throw std::invalid_argument("SDL_CreateRenderer");

            for (; active;) {
                SDL_Event event;
                while (SDL_PollEvent(&event)) {
                    int k;
                    switch (event.type) {
                    case SDL_QUIT:
                        active = false;
                        std::cout << "window closed" << std::endl;
                        break;
                    case SDL_KEYDOWN:
                        k = event.key.keysym.sym - SDLK_0;
                        if ((k >= 0) && (k < 10)) {
                            buttons_drv->trigger_button_down(k);
                        } else if (event.key.keysym.sym == SDLK_LEFT) {
                            view_x++;
                        } else if (event.key.keysym.sym == SDLK_RIGHT) {
                            view_x--;
                        } else if (event.key.keysym.sym == SDLK_UP) {
                            view_y++;
                        } else if (event.key.keysym.sym == SDLK_DOWN) {
                            view_y--;
                        }
                        break;
                    case SDL_KEYUP:
                        k = event.key.keysym.sym - SDLK_0;
                        if ((k >= 0) && (k < 10)) buttons_drv->trigger_button_up(k);
                        break;
                    }
                }

                SDL_SetRenderDrawColor(renderer.get(), 0, 0, 0, 255);
                SDL_RenderClear(renderer.get());

                SDL_SetRenderDrawColor(renderer.get(), 255, 255, 255, 255);
                distance_t s;
                std::list<distance_t> t;

                {
                    std::lock_guard<std::mutex> guard(list_mutex);
                    s = current_position;
                    t = movements_track;
                }
                draw_path(renderer, width, height, t);
                SDL_SetRenderDrawColor(renderer.get(), 255, 128, 128, 255);
                for (int i = 0; i < std::abs(s[2]); i++) {
                    SDL_RenderDrawPoint(renderer.get(), 
                     s[0] + view_x + i * (s[2] / std::abs(s[2]))*z_p_x/1000, 
                    -s[1] + view_y + i * (-s[2] / std::abs(s[2]))*z_p_y/1000 );
                }
                //std::cout << "step.. " << s[0] << ", " << s[1] << ", " << s[2] << std::endl;

                SDL_RenderPresent(renderer.get());
                SDL_Delay(33);
            }
        });
    }
    virtual ~video_sdl()
    {
        active = false;
        loop_thread.join();
    }
};
#else
class video_sdl
{
public:
    std::atomic<bool> active;
    void set_steps(const steps_t& st)
    {
    }

    video_sdl(configuration::global* cfg_, driver::low_buttons_fake* buttons_drv, int width = 640, int height = 480)
    {
        active = true;
    }
    virtual ~video_sdl()
    {
        active = false;
    }
};
#endif
/// end of visualization


void help_text(const std::vector<std::string>& args)
{
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
    std::cout << "\t\tprovide configuration file JSON" << std::endl;
    std::cout << std::endl;
    std::cout << "\t-C" << std::endl;
    std::cout << "\t\tdisplay current configuration in JSON format" << std::endl;
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
}


partitioned_program_t preprocess_program_parts(partitioned_program_t program_parts, const configuration::global &cfg) {
            block_t machine_state = {{'F', 0.5}};
            program_t prepared_program;

            for (auto& ppart : program_parts) {
                    if (ppart.size() != 0) {
                        if (ppart[0].count('M') == 0) {
                            //std::cout << "G PART: " << ppart.size() << std::endl;
                            switch ((int)(ppart[0]['G'])) {
                            case 0:
                                //ppart = g0_move_to_g1_sequence(ppart, cfg, machine_state);
                                //for (auto &e : ppart) e['G'] = 0;
                                //prepared_program.insert(prepared_program.end(), ppart.begin(), ppart.end());
                                //machine_state = last_state_after_program_execution(ppart,machine_state);
                                //break;
                            case 1:
                                ppart = g1_move_to_g1_with_machine_limits(ppart, cfg, machine_state);
                                prepared_program.insert(prepared_program.end(), ppart.begin(), ppart.end());
                                machine_state = last_state_after_program_execution(ppart, machine_state);
                                break;
                            case 4:
                                prepared_program.insert(prepared_program.end(), ppart.begin(), ppart.end());
                                break;
                            }
                        } else {
                            //std::cout << "M PART: " << ppart.size() << std::endl;
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

                //if (save_to_files_list.size() > 0) {
                //    std::cout << "SAVING prepared_program without DP FILE TO: " << (save_to_files_list.front()+".stage3") << std::endl;
                //    std::fstream f (save_to_files_list.front()+".stage3", std::fstream::out);
                //    f << back_to_gcode(group_gcode_commands(prepared_program)) << std::endl;
                //}
                prepared_program = optimize_path_douglas_peucker(prepared_program, 0.0125);
                program_parts = group_gcode_commands(remove_duplicate_blocks(prepared_program, {}));
                machine_state = {{'F', 0.5}};
    return program_parts;
}


int main(int argc, char** argv)
{
#ifdef HAVE_SDL2
    if (SDL_Init(SDL_INIT_VIDEO) != 0) throw std::invalid_argument("SDL_Init");
#endif

    using namespace std::chrono_literals;
    std::vector<std::string> args(argv, argv + argc);
    configuration::global cfg;
    cfg.load_defaults();

    bool raw_gcode = false; // should I push G commands directly, without adaptation to machine
    std::list < std::string> save_to_files_list;
    for (unsigned i = 1; i < args.size(); i++) {
        if ((args.at(i) == "-h") || (args.at(i) == "--help")) {
            help_text(args);
        } else if (args.at(i) == "-c") {
            i++;
            cfg.load(args.at(i));
        } else if (args.at(i) == "-C") {
            std::cout << cfg << std::endl;
        } else if (args.at(i) == "-s") {
            i++;
            save_to_files_list.push_back(args.at(i));
        } else if (args.at(i) == "--raw") {
            raw_gcode = true;
        } else if (args.at(i) == "-f") {
            using namespace raspigcd;
            using namespace raspigcd::hardware;

            std::shared_ptr<low_steppers> steppers_drv;
            std::shared_ptr<low_spindles_pwm> spindles_drv;
            std::shared_ptr<low_buttons> buttons_drv;

            std::shared_ptr<video_sdl> video;

            steps_t position_for_fake;
            try {
                auto rp = std::make_shared<driver::raspberry_pi_3>(cfg);
                steppers_drv = rp;
                spindles_drv = rp;
                buttons_drv = rp;
            } catch (...) {
                auto fk = std::make_shared<driver::inmem>();
                steppers_drv = fk;
                spindles_drv = std::make_shared<raspigcd::hardware::driver::low_spindles_pwm_fake>(
                    [](const int s_i, const double p_i) {
                        std::cout << "SPINDLE " << s_i << " POWER: " << p_i << std::endl;
                    });
                fk->on_enable_steppers = [](const std::vector<bool> m){
                    std::cout << "steppers: ";
                    for (auto e : m) {
                        std::cout << (e?"+":" ");
                    }
                    std::cout << ";" << std::endl;
                };
                auto buttons_fake_fake = std::make_shared<driver::low_buttons_fake>(10);
                buttons_drv = buttons_fake_fake;
                video = std::make_shared<video_sdl>(&cfg, buttons_fake_fake.get());
                fk->set_step_callback([&position_for_fake, &video](const steps_t& st) {
                    if (!(position_for_fake == st)) {
                        if (video.get() != nullptr) video->set_steps(st);
                    }
                    position_for_fake = st;
                });
            }
            //std::shared_ptr<driver::raspberry_pi_3> steppers_drv = std::make_shared<driver::raspberry_pi_3>(cfg);
            std::shared_ptr<motor_layout> motor_layout_ = motor_layout::get_instance(cfg);
            motor_layout_->set_configuration(cfg);

            std::shared_ptr<raspigcd::hardware::low_timers> timer_drv;
            switch (cfg.lowleveltimer) {
            case raspigcd::configuration::low_timers_e::WAIT_FOR:
                timer_drv = std::make_shared<hardware::driver::low_timers_busy_wait>();
                break;
            case raspigcd::configuration::low_timers_e::BUSY_WAIT:
                timer_drv = std::make_shared<hardware::driver::low_timers_wait_for>();
                break;
            case raspigcd::configuration::low_timers_e::FAKE:
                timer_drv = std::make_shared<hardware::driver::low_timers_fake>();
                break;
            }
            stepping_simple_timer stepping(cfg, steppers_drv, timer_drv);

            i++;
            std::ifstream gcd_file(args.at(i));
            if (!gcd_file.is_open()) throw std::invalid_argument("file should be opened");
            std::string gcode_text((std::istreambuf_iterator<char>(gcd_file)),
                std::istreambuf_iterator<char>());

            auto program = gcode_to_maps_of_arguments(gcode_text);
            double previous_feedrate_g0 = 0.1;
            for (auto& p : program) {
                if (p.count('G')) {
                    if (p['G'] == 0) {
                    p['F'] = *std::max_element(
                        std::begin(cfg.max_velocity_mm_s), 
                        std::end(cfg.max_velocity_mm_s));
                    } else if (p['G'] == 1) {
                        if (p.count('F')) {
                            previous_feedrate_g0 = p['F'];
                        } else {
                            p['F'] = previous_feedrate_g0;
                        }
                    }
                } 
            }
            if (save_to_files_list.size() > 0) {
                std::cout << "SAVING RAW FILE TO: " << (save_to_files_list.front()+".stage0") << std::endl;
                std::fstream f (save_to_files_list.front()+".stage0", std::fstream::out);
                f << back_to_gcode(group_gcode_commands(program)) << std::endl;
            }
            program = optimize_path_douglas_peucker(program, 0.0125);
            if (save_to_files_list.size() > 0) {
                std::cout << "SAVING optimize_path_douglas_peucker FILE TO: " << (save_to_files_list.front()+".stage1") << std::endl;
                std::fstream f (save_to_files_list.front()+".stage1", std::fstream::out);
                f << back_to_gcode(group_gcode_commands(program)) << std::endl;
            }

            auto program_parts = group_gcode_commands(program);
            block_t machine_state = {{'F', 0.5}};
            program_parts = insert_additional_nodes_inbetween(program_parts, machine_state, cfg);
            if (save_to_files_list.size() > 0) {
                std::cout << "SAVING insert_additional_nodes_inbetween FILE TO: " << (save_to_files_list.front()+".stage2") << std::endl;
                std::fstream f (save_to_files_list.front()+".stage2", std::fstream::out);
                f << back_to_gcode(program_parts) << std::endl;
            }

            if (!raw_gcode) {
                program_parts = preprocess_program_parts(program_parts,cfg);
            } // if prepare paths
            std::atomic<int> break_execution_result = -1;
            std::function<void(int, int)> on_pause_execution;
            auto on_resume_execution = [&stepping, buttons_drv, &on_pause_execution, &break_execution_result](int k, int s) {
                if (s == 1) {
                    std::cout << "######## resume" << std::endl;
                    break_execution_result = 1;
                    buttons_drv->on_key(k, on_pause_execution);
                }
            };
            auto on_stop_execution = [&stepping, buttons_drv, &break_execution_result](int k, int s) {
                if ((k == 1) && (s == 1)) {
                    break_execution_result = 0;
                    stepping.terminate(1000);
                }
            };
            on_pause_execution = [&stepping, buttons_drv, &on_resume_execution, &break_execution_result](int k, int s) {
                std::cout << "######## Key " << k << " is " << ((s == 0) ? "UP" : "DOWN") << std::endl;
                if ((k == 0) && (s == 1) && (break_execution_result == -1)) {
                    break_execution_result = -1;
                    buttons_drv->on_key(k, on_resume_execution);
                    stepping.terminate(1000);
                }
            };

            for (unsigned int i = 0; i < buttons_drv->keys_state().size(); i++) {
                if (i == 0) {
                    buttons_drv->on_key(i, on_pause_execution);
                } else if (i == 1) {
                    buttons_drv->on_key(i, on_stop_execution);
                } else {
                    buttons_drv->on_key(i, [&stepping, buttons_drv](int k, int s) {
                        std::cout << "Key " << k << " is " << ((s == 0) ? "UP" : "DOWN") << std::endl;
                    });
                }
            }


            std::cout << "STARTING" << std::endl;

            if (save_to_files_list.size() > 0) {
                std::cout << "SAVING PREPROCESSED FILE TO: " << save_to_files_list.front() << std::endl;
                std::fstream f (save_to_files_list.front(), std::fstream::out);
                save_to_files_list.pop_front();
                f << back_to_gcode(program_parts) << std::endl;
            }
            machine_state = {{'F', 0.5}};
            std::map<int, double> spindles_status;
            long int last_spindle_on_delay = 7000;
            for (auto& ppart : program_parts) {
                
                //std::cout << "Put part: " << ppart.size() << std::endl;
                if (ppart.size() != 0) {
                    if (ppart[0].count('M') == 0) {
                        //std::cout << "G PART: " << ppart.size() << std::endl;
                        switch ((int)(ppart[0]['G'])) {
                        case 0:
                        case 1:
                        case 4:
                            auto machine_state_prev = machine_state;

                            auto time0 = std::chrono::high_resolution_clock::now();
                            auto m_commands = converters::program_to_steps(ppart, cfg, *(motor_layout_.get()),
                                machine_state, [&machine_state](const block_t& result) {
                                    machine_state = result;
                                    // std::cout << "____ program_to_steps:";
                                    // for (auto& s : machine_state) {
                                    //     std::cout << s.first << ":" << s.second << " ";
                                    // }
                                    // std::cout << std::endl;
                                });
                            auto time1 = std::chrono::high_resolution_clock::now();

                            double dt = std::chrono::duration<double, std::milli>(time1-time0).count();
                            std::cout << "calculations took " << dt << " milliseconds; have " << m_commands.size() << " steps to execute" << std::endl;
                            try {
                                stepping.exec(m_commands, [motor_layout_,&spindles_status, timer_drv, spindles_drv, &break_execution_result,machine_state_prev,last_spindle_on_delay](auto steps_from_origin, auto tick_n) -> int {
                                    std::cout << "break at " << tick_n << " tick" << std::endl;
                                    steps_from_origin = steps_from_origin + motor_layout_->cartesian_to_steps(block_to_distance_t(machine_state_prev));
                                    std::cout << "Position: " << motor_layout_->steps_to_cartesian(steps_from_origin) << std::endl;
                                    for (auto e : spindles_status) {
                                        spindles_drv->spindle_pwm_power(e.first, 0);
                                    }
                                    while (break_execution_result < 0) {
                                        timer_drv->wait_us(10000);
                                    }
                                    if ((int)(break_execution_result) == 1) {
                                        for (auto e : spindles_status) {
                                            spindles_drv->spindle_pwm_power(e.first, e.second);
                                            using namespace std::chrono_literals;
                                            std::cout << "wait for spindle..." << std::endl;
                                            std::this_thread::sleep_for(std::chrono::milliseconds(last_spindle_on_delay));
                                            std::cout << "wait for spindle... OK" << std::endl;
//                                            machine_state['X'] = ;
//                                            machine_state['Y'] = ;
//                                            machine_state['Z'] = ;
//                                            machine_state['A'] = ;
                                        }
                                    }
                                    int r = break_execution_result;
                                    break_execution_result = -1;
                                    return r;
                                });
                            } catch (...) {
                                steppers_drv->enable_steppers({false});
                                spindles_drv->spindle_pwm_power(0, 0.0);
                            }
                            break;
                        }
                    } else {
                        auto wait_for_component_to_start = [](auto m, int t = 3000) {
                            if (m.count('P') == 1) {
                                t = m.at('P');
                            } else if (m.count('X') == 1) {
                                t = 1000 * m.at('X');
                            }
                            if (t > 0)
                                std::this_thread::sleep_for(std::chrono::milliseconds((int)t));
                            return t;
                        };
                        for (auto& m : ppart) {
                            switch ((int)(m['M'])) {
                            case 17:
                                steppers_drv->enable_steppers({true});
                                wait_for_component_to_start(m,200);
                                break;
                            case 18:
                                steppers_drv->enable_steppers({false});
                                wait_for_component_to_start(m,200);
                                break;
                            case 3:
                                spindles_status[0] = 1.0;
                                spindles_drv->spindle_pwm_power(0, spindles_status[0]);
                                last_spindle_on_delay = wait_for_component_to_start(m,3000);
                                break;
                            case 5:
                                spindles_status[0] = 0.0;
                                spindles_drv->spindle_pwm_power(0, spindles_status[0]);
                                wait_for_component_to_start(m,3000);
                                break;
                            }
                        }
                    }
                }
                if (video.get() != nullptr) {
                    if (!(video->active)) {
                        stepping.terminate();
                        break;
                    }
                }
                std::string s = "";
                for (auto e: machine_state) {
                    s = s + ((s.size())?" ":"") + e.first+std::to_string(e.second);
                }
                std::cout << s << std::endl;
            }
            std::cout << "FINISHED" << std::endl;
        }
    }
#ifdef HAVE_SDL2
    SDL_Quit();
#endif

    return 0;
}
