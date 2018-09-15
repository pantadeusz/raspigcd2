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

int executor_sim_t::execute(const std::vector<executor_command_t> &commands)
{
    configuration_t conf = configuration_t::get();
    unsigned long tn = 0;
    distance_t position(0,0,0,0);
    distance_t target_position(0,0,0,0);
    distance_t velocity(0,0,0,0);
    
    distance_t forcecoeff(1000.0,1000.0,1000.0,1000.0);
    distance_t frictioncoeff(10.5,10.5,10.5,10.5);
    double dtnn = 50;
    double dt = conf.tick_duration / dtnn;
    double modder = 0;
    for (auto c : commands)
    {
        int dir[4] = {0,0,0,0};
        // step direction
        dir[0] = c.b.step0*(c.b.dir0*2 - 1);
        dir[1] = c.b.step1*(c.b.dir1*2 - 1);
        dir[2] = c.b.step2*(c.b.dir2*2 - 1);
        dir[3] = c.b.step3*(c.b.dir3*2 - 1);
        for (int i:{0,1,2,3}) steps_from_origin_[i] += dir[i];
        
        for (int i = 0; i < 4; i++) {
            target_position[i] = ((double)steps_from_origin_[i])/conf.hardware.steppers[i].stepsPerMm;
        }

        //std::cerr << "dir 0 " << (1.0/conf.hardware.steppers[0].stepsPerMm) << std::endl;
        for (int dtn = 0; dtn < dtnn; dtn++) {
            for (int i = 0; i < 4; i++) {
                double a = 0;
                double direction = target_position[i]-position[i];
                if (std::abs(direction) > (32.1/conf.hardware.steppers[i].stepsPerMm)) { // krok zgubiony
                    a = 0;// 0.1*(1/(std::abs(direction) > (1.5/conf.hardware.steppers[i].stepsPerMm)))*dt*direction/std::abs(direction);
                } else {
                    double azz = std::abs(direction/(conf.hardware.steppers[i].stepsPerMm*dt) );
                    //modder = modder*0.99 + std::abs(direction/(conf.hardware.steppers[i].stepsPerMm*dt) )*0.01 ;
                    modder = std::abs(direction/(conf.hardware.steppers[i].stepsPerMm*dt) )*0.01 ;
                    
                    //if (modder != 0) 
                    //std::cerr << "modder:: " << modder << " " << azz << std::endl;
                    //if (modder < 1) modder = std::sqrt(modder);
                    //modder = modder;//*std::log2(modder);
                    //if (modder < 10) modder = 10;
                    //modder = std::sqrt(modder);
                    //if (modder > 60) modder = 60;
                    //modder = 1;
                    //if (std::abs(direction) < (0.1/conf.hardware.steppers[i].stepsPerMm)) {
                        if (direction != 0) a = (1.0/std::sqrt(modder+1))*dt*direction/std::abs(direction);    
                    //} else {
                    //    if (direction != 0) a = 0.2*dt*direction/std::abs(direction);
                    //}
                }
                velocity[i] = (velocity[i] + a)*0.9999;// - (velocity[i]*std::abs(velocity[i]))*dt*dt*(0.01*0.01);
                //velocity[i] = velocity[i] + a - (velocity[i])*dt*0.1;
                position[i] = position[i] + velocity[i];
            }

            std::cout << "" << (((double)(tn) / 1000000.0) + dt*dtn) << 
            //" " << (target_position[0] - position[0]) << 
            //" " << (target_position[1] - position[1]) << 
            //" " << (target_position[2] - position[2]) << 
            //" " << (target_position[3] - position[3]) << "\n";
            " " << position[0] <<
            " " << position[1] <<
            " " << position[2] <<
            " " << position[3] << "\n";
            //" " << target_position[0] <<
            //" " << target_position[1] <<
            //" " << target_position[2] <<
            //" " << target_position[3] << "\n";

        }
        std::cout << "" << ((double)(tn) / 1000000.0) << 
                " " << (steps_from_origin_[0]/conf.hardware.steppers[0].stepsPerMm) << 
                " " << (steps_from_origin_[1]/conf.hardware.steppers[1].stepsPerMm) << 
                " " << (steps_from_origin_[2]/conf.hardware.steppers[2].stepsPerMm) << 
                " " << (steps_from_origin_[3]/conf.hardware.steppers[3].stepsPerMm) << "\n";
        tn += (conf.tick_duration*1000000);
        //std::cout << "" << ((double)(tn) / 1000000.0) << " " << position[0] << " " << position[1] << " " << position[2] << " " << position[3] << "\n";
    }
    std::cout << "tick duration = " << conf.tick_duration << std::endl;
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
