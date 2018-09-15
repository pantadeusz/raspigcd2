/*

    Raspberry Pi G-CODE interpreter
    Copyright (C) 2018  Tadeusz Puźniakowski puzniakowski.pl

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURsteps_from_origin_E.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

*/

#include <executor_sim_t.hpp>
#include <distance_t.hpp>

#include <chrono>

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <cmath>

namespace raspigcd
{

executor_sim_t::executor_sim_t()
{
    for (auto &p: steps_from_origin_) p = 0;
    enable(false);
}

executor_sim_t::~executor_sim_t()
{
}

executor_sim_t &executor_sim_t::get() {
    static executor_sim_t instance;
    return instance;
}

//plot 'file.txt' using 1:4 w lines, '' using 1:8 w lines

int executor_sim_t::execute(const std::vector<executor_command_t> &commands)
{

    // using SI coordinates: m, m/s, kg, N

    configuration_t conf = configuration_t::get();
    distance_t position(0,0,0,0); // current position 
    distance_t target_position(0,0,0,0); // calculated as simulation in meters
    distance_t velocity(0,0,0,0); // current velocity in m/s
    distance_t force(0,0,0,0); // current force N
    distance_t force_from_motor(10,10,10,10); // maximal force for given motor
    distance_t friction_coeff(0.0001,0.0001,0.0001,0.0001); // maximal force for given motor
    distance_t mass(10,10,10,10); // current mass in kg
    distance_t break_distance(0.01,0.01,0.01,0.01); // 1mm 



    distance_t friction(0,0,0,0); // current friction force

    double dtnn = 4; // simulation multiplier
    double dt = conf.tick_duration / dtnn; // dt for simulation
    std::cerr << "commands count = " << commands.size() << std::endl;
    int tick_i = 0;
    for (auto c : commands)
    {
        int dir[4] = {0,0,0,0};
        // step direction
        dir[0] = c.b.step0*(c.b.dir0*2 - 1);
        dir[1] = c.b.step1*(c.b.dir1*2 - 1);
        dir[2] = c.b.step2*(c.b.dir2*2 - 1);
        dir[3] = c.b.step3*(c.b.dir3*2 - 1);
        for (int i:{0,1,2,3}) {
            steps_from_origin_[i] += dir[i];
            target_position[i] = ((double)steps_from_origin_[i])/(conf.hardware.steppers[i].stepsPerMm*1000.0); // in meters
        }

        //std::cerr << "dir 0 " << (1.0/conf.hardware.steppers[0].stepsPerMm) << std::endl;
        for (int dtn = 0; dtn < dtnn; dtn++) {
            for (int i = 0; i < 4; i++) {
                double direction = target_position[i]-position[i];
                if (std::abs(direction) > break_distance[i]) {
                    position[i] = target_position[i]; // step lost - JITTER
                    direction = target_position[i]-position[i];
                }
                if (std::abs(direction) > 0.0000001) {
                    force[i] = force_from_motor[i]*direction/std::abs(direction)*(1.0 - std::pow(std::abs(direction)/break_distance[i],2));
                } else force[i] = 0;
                friction[i] = -((velocity[i]>0)?1.0:-1.0)*std::abs(velocity[i])*friction_coeff[i]/dt;
                force[i] += friction[i];
                velocity[i] = velocity[i] + (force[i]/mass[i])*dt;
                position[i] = position[i] + velocity[i]*dt;
            }

            std::cout << "" << (((double)(tick_i) * conf.tick_duration) + dt*dtn) <<
            " " << (target_position[0]) <<
            " " << (target_position[1]) <<
            " " << (target_position[2]) <<
            " " << (target_position[3]) <<
            " " << (position[0]) <<
            " " << (position[1]) <<
            " " << (position[2]) <<
            " " << (position[3]) <<
            " " << (velocity[0]) <<
            " " << (velocity[1]) <<
            " " << (velocity[2]) <<
            " " << (velocity[3]) <<
            " " << (force[0]) <<
            " " << (force[1]) <<
            " " << (force[2]) <<
            " " << (force[3]) <<
            " " << (friction[0]) <<
            " " << (friction[1]) <<
            " " << (friction[2]) <<
            " " << (friction[3]) <<
            "\n";

        }
        tick_i++;
    }
    std::cerr << "tick duration = " << conf.tick_duration << std::endl;
    std::cerr << "end time = " << (tick_i*conf.tick_duration) << std::endl;
    return 0;
}

void executor_sim_t::enable(bool)
{
}

void executor_sim_t::set_position(const steps_t &steps) {
    steps_from_origin_ = steps;
}
steps_t executor_sim_t::get_position() const {
    return steps_from_origin_;
}


} // namespace raspigcd
