/*
    Raspberry Pi G-CODE interpreter
    Copyright (C) 2018  Tadeusz Puźniakowski puzniakowski.pl
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <distance_t.hpp>
#include <executor_sim_t.hpp>

#include <chrono>

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace raspigcd {

std::mutex executor_sim_t::_mutex_steps_from_origin;

executor_sim_t::executor_sim_t(configuration_t& cfg)
{
    _cfg = &cfg;
    debug_callback = []( double , // T,
                         distance_t , // target_position,
                         distance_t , // position,
                         distance_t , // velocity,
                         distance_t , // force,
                         distance_t , // friction,
                         int  // num_empty_ticks
                         ) {
        };
    for (auto& p : _steps_from_origin)
        p = 0;
    enable(false);
}

executor_sim_t::~executor_sim_t()
{
}

executor_sim_t& executor_sim_t::get(configuration_t& c_)
{
    static executor_sim_t instance(c_);
    return instance;
}

//plot 'file.txt' using 1:4 w lines, '' using 1:8 w lines

int executor_sim_t::execute(const std::vector<executor_command_t>& commands)
{
    // using SI coordinates: m, m/s, kg, N


    distance_t position(0, 0, 0, 0);                       // current position
    distance_t target_position(0, 0, 0, 0);                // calculated as simulation in meters
    distance_t velocity(0, 0, 0, 0);                       // current velocity in m/s
    distance_t force(0, 0, 0, 0);                          // current force N
    distance_t force_from_motor(50, 50, 50, 50);           // maximal force for given motor
    distance_t friction_coeff(0.002, 0.002, 0.002, 0.002); // maximal force for given motor
    distance_t mass(9, 9, 9, 9);                           // current mass in kg
    distance_t break_distance(0.01, 0.01, 0.01, 0.01);     // 1mm

    distance_t friction(0, 0, 0, 0); // current friction force

    double dtnn = 4;                        // simulation multiplier
    double dt = _cfg->tick_duration / dtnn; // dt for simulation
    std::cerr << "commands count = " << commands.size() << std::endl;
    int tick_i = 0;
    int num_empty_ticks = 0;

    auto prev_position_steps = _steps_from_origin;
    for (int i = 0; i < DEGREES_OF_FREEDOM; i++)
        position[i] = ((double)_steps_from_origin[i]) / (_cfg->hardware.steppers[i].steps_per_m()); // in meters
    for (auto c : commands) {
        int rpt = c.cmnd.repeat; // 0 means that we execute it once
        do {
            int dir[4] = {0, 0, 0, 0};
            // step direction
            for (auto i : {0, 1, 2, 3})
                dir[i] = c.b[i].step * (c.b[i].dir * 2 - 1);
            for (int i : {0, 1, 2, 3}) {
                _steps_from_origin[i] += dir[i];
                target_position[i] = ((double)_steps_from_origin[i]) / (_cfg->hardware.steppers[i].steps_per_m()); // in meters
            }

            //std::cerr << "dir 0 " << (1.0/_cfg->hardware.steppers[0].stepsPerMm) << std::endl;
            for (int dtn = 0; dtn < dtnn; dtn++) {
                for (int i = 0; i < 4; i++) {
                    double direction = target_position[i] - position[i];
                    if (std::abs(direction) > break_distance[i]) {
                        position[i] = target_position[i]; // step lost - JITTER
                        direction = target_position[i] - position[i];
                    }
                    if (std::abs(direction) > 0.0000001) {
                        force[i] = force_from_motor[i] * direction / std::abs(direction) * (1.0 - std::pow(std::abs(direction) / break_distance[i], 2));
                    } else
                        force[i] = 0;
                    friction[i] = -((velocity[i] > 0) ? 1.0 : -1.0) * std::abs(velocity[i]) * friction_coeff[i] / dt;
                    force[i] += friction[i];
                    velocity[i] = velocity[i] + (force[i] / mass[i]) * dt;
                    position[i] = position[i] + velocity[i] * dt;
                }
                if (!(prev_position_steps == _steps_from_origin)) {
                    debug_callback((((double)(tick_i)*_cfg->tick_duration) + dt * dtn),
                        target_position,
                        position,
                        velocity,
                        force,
                        friction,
                        num_empty_ticks);
                    num_empty_ticks = 0;
                } else {
                    num_empty_ticks++;
                }
                prev_position_steps = _steps_from_origin;
            }
            tick_i++;
        } while ((rpt--) > 0);
    }
    std::cerr << "tick duration = " << _cfg->tick_duration << std::endl;
    std::cerr << "end time = " << (tick_i * _cfg->tick_duration) << std::endl;
    return 0;
}

void executor_sim_t::enable(bool)
{
}

void executor_sim_t::set_position(const steps_t& steps)
{
    std::lock_guard<std::mutex> lock(_mutex_steps_from_origin);
    _steps_from_origin = steps;
}
steps_t executor_sim_t::get_position() const
{
    std::lock_guard<std::mutex> lock(_mutex_steps_from_origin);
    return _steps_from_origin;
}

} // namespace raspigcd
