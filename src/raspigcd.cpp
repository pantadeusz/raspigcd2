#include <executor_t.hpp>
#include <configuration_t_json.hpp>
#include <vector>
#include <string>
#include <iostream>

using namespace raspigcd;
int steps_to_do(const steps_t &steps_, const steps_t &destination_steps_) {
    int ret = 0;
    for (int i = 0; i < steps_.size(); i++) {
        ret = std::max(std::abs(steps_[i]-destination_steps_[i]),ret);
    }
    return ret;
}

/**
 * @brief generates steps to reach given destination steps
 * @arg steps_ current steps count
 * @arg destination_steps_ desired steps count
 */
std::vector<executor_command_t> chase_steps(const steps_t &steps_, steps_t destination_steps_) {
    auto &cfg = configuration_t::get();
    std::vector<executor_command_t> ret;
    auto steps = steps_;
    executor_command_t executor_command;
    do {
        executor_command.v = 0;
        for (int i = 0; i < steps.size(); i++) {
            executor_command.b[i].dir = ((destination_steps_[i] > steps[i])?1:0)*((cfg.hardware.steppers[i].direction_reverse)?-1:1);
            executor_command.b[i].step = (destination_steps_[i] - steps[i])?1:0;
            if (destination_steps_[i] > steps[i]) steps[i]++;
            else if (destination_steps_[i] < steps[i]) steps[i]--;
        }
        ret.push_back(executor_command);
    } while (steps_to_do(steps,destination_steps_)>0);
    return ret;
}

std::vector<executor_command_t>  genSinWave() {
    static double amplitude = 10; // in milimeters
    static double T = 3;
    auto &cfg = configuration_t::get();

    std::vector<executor_command_t> executor_commands;
    steps_t steps;

    int axis = 2;
    for (int i = 0; i < 100; i++) {
         executor_command_t e;
         e.v=0;
         executor_commands.push_back(e);
    }

//    steps[axis] = cfg.hardware.steppers[axis].stepsPerMm*std::cos(0)*amplitude;
    for (double t = 0.0; t < T;t+=cfg.tick_duration) {
        steps_t new_steps;
        new_steps[axis] = cfg.hardware.steppers[axis].stepsPerMm*std::cos(t*3.141592653589793238462643*10)*amplitude*std::sin(3.141592653589793238462643*t/T);
        auto st = chase_steps(steps,new_steps);
        executor_commands.insert(executor_commands.end(),st.begin(), st.end());
        steps = new_steps;
    }
    std::cerr << "generated " << executor_commands.size() << "steps" << std::endl;
    return executor_commands;
}


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

    executor.set_position({0,0,0,0});
    for (int avv = 30; avv >= 10; avv-=5) {
    for (int i = 0; i < cfg.hardware.steppers[2].stepsPerMm*5; i++) {
        executor_command_t executor_command;
        executor_command.v = 0;
        executor_command.b[2].dir = 1;

        executor_command.b[2].step = 0;
        for (int j = 0; j < avv; j++) executor_commands.push_back(executor_command);

        executor_command.b[2].step = 1;
        executor_commands.push_back(executor_command);
    }
    for (int i = 0; i < cfg.hardware.steppers[2].stepsPerMm*5; i++) {
        executor_command_t executor_command;
        executor_command.v = 0;
        executor_command.b[2].dir = 0;

        executor_command.b[2].step = 0;
        for (int j = 0; j < avv; j++) executor_commands.push_back(executor_command);

        executor_command.b[2].step = 1;
        executor_commands.push_back(executor_command);
    }
    }
    executor.enable(true);
    //executor.execute(executor_commands);
    executor.execute(genSinWave());
    executor.enable(false);

    return 0;
}