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

    video_sdl(configuration::global* cfg_, driver::low_buttons_fake* buttons_drv, int width = 640, int height = 480)
    {
        active = true;
        cfg = cfg_;
        ml = motor_layout::get_instance(*cfg);

        movements_track.push_back({0, 0, 0, 0});
        loop_thread = std::thread([this, width, height, buttons_drv]() {
            std::cout << "loop thread..." << std::endl;

            SDL_Window* win = SDL_CreateWindow("Witaj w Swiecie",
                SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                width, height, SDL_WINDOW_SHOWN);
            if (win == nullptr) throw std::invalid_argument("SDL_CreateWindow - error");
            window = std::shared_ptr<SDL_Window>(win, [](SDL_Window* ptr) {
                SDL_DestroyWindow(ptr);
            });

            SDL_Renderer* ren = SDL_CreateRenderer(window.get(), -1, SDL_RENDERER_ACCELERATED);
            if (ren == nullptr) throw std::invalid_argument("SDL_CreateRenderer");
            renderer = std::shared_ptr<SDL_Renderer>(ren, [](SDL_Renderer* ptr) {
                SDL_DestroyRenderer(ptr);
            });
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
                        if ((k >= 0) && (k < 10)) buttons_drv->trigger_button_down(k);
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
                for (auto e : t) {
                    if (e[2] <= 0) {
                        SDL_SetRenderDrawColor(renderer.get(), 255 - (e[2] * 255 / 5), 255, 255, 255);
                        SDL_RenderDrawPoint(renderer.get(), e[0] + width / 2, e[1] + height / 2);
                    }
                }
                SDL_SetRenderDrawColor(renderer.get(), 255, 128, 128, 255);
                for (int i = 0; i < std::abs(s[2]); i++) {
                    SDL_RenderDrawPoint(renderer.get(), s[0] + width / 2 - i * (s[2] / std::abs(s[2])), s[1] + i * (s[2] / std::abs(s[2])) + height / 2);
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
}

int main(int argc, char** argv)
{
#ifdef HAVE_SDL2
    //std::cout << "WITH SDL!!! " << std::endl;
    if (SDL_Init(SDL_INIT_VIDEO) != 0) throw std::invalid_argument("SDL_Init");
#endif

    using namespace std::chrono_literals;
    std::vector<std::string> args(argv, argv + argc);
    configuration::global cfg;
    cfg.load_defaults();

    bool raw_gcode = false; // should I push G commands directly, without adaptation to machine

    for (unsigned i = 1; i < args.size(); i++) {
        if ((args.at(i) == "-h") || (args.at(i) == "--help")) {
            help_text(args);
        } else if (args.at(i) == "-c") {
            i++;
            cfg.load(args.at(i));
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
            for (auto& p : program) {
                if ((p.count('G')) && (p['G'] == 0))
                    p['F'] = *std::max_element(std::begin(cfg.max_velocity_mm_s), std::end(cfg.max_velocity_mm_s));
            }
            auto program_parts = group_gcode_commands(program);
            std::cout << "Initial GCODE: " << std::endl
                      << back_to_gcode(program_parts) << std::endl;
            block_t machine_state = {{'F', 0.5}};
            program_parts = insert_additional_nodes_inbetween(program_parts, machine_state, cfg);
            std::cout << "Initial GCODE w additional parts: " << std::endl
                      << back_to_gcode(program_parts) << std::endl;

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
                                machine_state = last_state_after_program_execution(ppart, machine_state);
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

                std::cout << "Prepared GCODE NO GROUPING INIT: " << std::endl
                          << back_to_gcode({prepared_program}) << std::endl;
                std::cout << "Prepared GCODE INIT: " << std::endl
                          << back_to_gcode(group_gcode_commands(prepared_program)) << std::endl;
                std::cout << "Prepared GCODE NO DUPLICATES: " << std::endl
                          << back_to_gcode({remove_duplicate_blocks(prepared_program, {})}) << std::endl;
                program_parts = group_gcode_commands(remove_duplicate_blocks(prepared_program, {}));
                //program_parts = group_gcode_commands(prepared_program);
                std::cout << "Prepared GCODE: " << std::endl
                          << back_to_gcode(program_parts) << std::endl;
                machine_state = {{'F', 0.5}};
            } // if prepare paths


            for (int i = 0; i < buttons_drv->keys_state().size(); i++) {
                std::cout << "Handler for key " << i << std::endl;
                buttons_drv->on_key(i, [&stepping](int k, int s) {
                    std::cout << "Key " << k << " is " << ((s == 0) ? "UP" : "DOWN") << std::endl;
                    if ((k == 0) && (s == 1)) stepping.terminate(1000);
                });
            }


            std::cout << "STARTING" << std::endl;

            machine_state = {{'F', 0.5}};
            for (auto& ppart : program_parts) {
                std::cout << "Put part: " << ppart.size() << std::endl;
                if (ppart.size() != 0) {
                    if (ppart[0].count('M') == 0) {
                        //std::cout << "G PART: " << ppart.size() << std::endl;
                        switch ((int)(ppart[0]['G'])) {
                        case 0:
                        case 1:
                        case 4:
                            auto m_commands = converters::program_to_steps(ppart, cfg, *(motor_layout_.get()),
                                machine_state, [&machine_state](const block_t& result) {
                                    machine_state = result;
                                    // std::cout << "____ program_to_steps:";
                                    // for (auto& s : machine_state) {
                                    //     std::cout << s.first << ":" << s.second << " ";
                                    // }
                                    // std::cout << std::endl;
                                });
                            try {
                            stepping.exec(m_commands);
                            } catch (...) {
                                std::cout << "exceptional exception" << std::endl;
                            }
                            //std::list<steps_t> steps = hardware_commands_to_steps(m_commands);
                            //std::cout << "steps: " << motor_layout_->steps_to_cartesian(steps.back()) << std::endl;
                            break;
                        }
                    } else {
                        //std::cout << "M PART: " << ppart.size() << std::endl;
                        for (auto& m : ppart) {
                            switch ((int)(m['M'])) {
                            case 17:
                                steppers_drv->enable_steppers({true});
                                break;
                            case 18:
                                steppers_drv->enable_steppers({false});
                                break;
                            case 3:
                                spindles_drv->spindle_pwm_power(0, 1.0);
                                break;
                            case 5:
                                spindles_drv->spindle_pwm_power(0, 0.0);
                                break;
                            }
                        }
                    }
                }
                //std::cout << "[MS] Machine state:  ";
                //for (auto& s : machine_state) {
                //    std::cout << s.first << ":" << s.second << " ";
                //}
                //std::cout << "  OK" << std::endl;
                if (video.get() != nullptr) {
                    if (!(video->active)) {
                        std::cout << "video is not active" << std::endl;
                        stepping.terminate();
                        break;
                    }
                }
            }
            std::cout << "FINISHED" << std::endl;
        }
    }
#ifdef HAVE_SDL2
    SDL_Quit();
#endif

    return 0;
}
