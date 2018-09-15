#include <executor_t.hpp>
#include <configuration_t_json.hpp>
#include <vector>
#include <string>
#include <iostream>

using namespace raspigcd;
/*
std::vector<executor_command_t> chase_steps(steps_t steps, steps_t destination_steps, int axis) {
    std::vector<executor_command_t> ret;
    
    return ret;
}

std::vector<executor_command_t>  genSinWave() {
        auto &cfg = configuration_t::get().load_defaults();

    std::vector<executor_command_t> executor_commands;

for (double t = 0.0; t < 3;t+=cfg.tick_duration) {
        //if (v < 500) {
            v = (v + a * dt) * 0.999;
        //} else v = 500;
        p = p + v*dt;
        executor_command_t executor_command;
        executor_command.v = 0;
        executor_command.b.dir2 = 1;

        if (p > (((double)ticks)/cfg.hardware.steppers[2].stepsPerMm)) {
            executor_command.b.step2 = 1;
            ticks++;
            std::cerr << " ++ " << p << " " << v  << "  -> "  << (((double)ticks)/cfg.hardware.steppers[2].stepsPerMm) << std::endl;
        } else {
            executor_command.b.step2 = 0;
            std::cerr << "    " << p << " " << v  << "  -> "  << (((double)ticks)/cfg.hardware.steppers[2].stepsPerMm) << std::endl;
        }

        executor_commands.push_back(executor_command);
    }
    return executor_commands;
}

*/

int main(int argc, char **argv)
{
    std::vector<std::string> args(argv, argv + argc);

    auto &cfg = configuration_t::get().load_defaults();
    std::cout << cfg << std::endl;
    executor_t &executor = executor_t::get();
    std::vector<executor_command_t> executor_commands;
    executor_commands.reserve(cfg.hardware.steppers[2].stepsPerMm*2*10);
    double v = 0.0;
    double dt = cfg.tick_duration;
    double a = 5000;
    int ticks = 0;
/*    for (double p = 0.0; p < 3;) {
        //if (v < 500) {
            v = (v + a * dt) * 0.999;
        //} else v = 500;
        p = p + v*dt;
        executor_command_t executor_command;
        executor_command.v = 0;
        executor_command.b.dir2 = 1;

        if (p > (((double)ticks)/cfg.hardware.steppers[2].stepsPerMm)) {
            executor_command.b.step2 = 1;
            ticks++;
            std::cerr << " ++ " << p << " " << v  << "  -> "  << (((double)ticks)/cfg.hardware.steppers[2].stepsPerMm) << std::endl;
        } else {
            executor_command.b.step2 = 0;
            std::cerr << "    " << p << " " << v  << "  -> "  << (((double)ticks)/cfg.hardware.steppers[2].stepsPerMm) << std::endl;
        }

        executor_commands.push_back(executor_command);
    } */

    for (int avv = 500; avv >= 498; avv--) {
    for (int i = 0; i < cfg.hardware.steppers[2].stepsPerMm*5; i++) {
        executor_command_t executor_command;
        executor_command.v = 0;
        executor_command.b.dir2 = 1;
        executor_command.b.step2 = 1;
        executor_commands.push_back(executor_command);
        executor_command.b.step2 = 0;
        for (int j = 0; j < avv; j++) executor_commands.push_back(executor_command);
    }
    for (int i = 0; i < cfg.hardware.steppers[2].stepsPerMm*5; i++) {
        executor_command_t executor_command;
        executor_command.v = 0;
        executor_command.b.dir2 = 0;
        executor_command.b.step2 = 1;
        executor_commands.push_back(executor_command);
        executor_command.b.step2 = 0;
        for (int j = 0; j < avv; j++) executor_commands.push_back(executor_command);
    }
    }
    executor.enable(true);
    executor.execute(executor_commands);
    executor.enable(false);

    return 0;
}