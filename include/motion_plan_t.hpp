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



#ifndef __RASPIGCD_MOTION_PLAN_HPP___
#define __RASPIGCD_MOTION_PLAN_HPP___

#include <configuration_t.hpp>
#include <motor_layout_t.hpp>
#include <distance_t.hpp>
#include <step_sequence_executor_t.hpp>
#include <steps_t.hpp>

#include <memory>
#include <list>
#include <vector>
#include <tuple>
#include <cmath>


namespace raspigcd {


class motion_fragment_t
{
public:
    distance_t source, destination;
    double max_velocity;
    std::shared_ptr<double> prev_speed;
    std::shared_ptr<double> next_speed;
};


class motion_plan_t
{
protected:
    std::list<motion_fragment_t> _motion_fragments;
    distance_t _current_position;

    configuration_t* _cfg;
    motor_layout_t* motor_layout;
protected:



public:
    const std::vector<executor_command_t> get_motion_plan() const;
    const steps_t get_steps() const;
    const distance_t get_position() const;


static int steps_to_do(const steps_t& steps_, const steps_t& destination_steps_);

/**
 * @brief generates steps to reach given destination steps
 * @arg steps_ current steps count
 * @arg destination_steps_ desired steps count
 */
static  std::vector<executor_command_t> chase_steps(const steps_t& steps_, steps_t destination_steps_);
    motion_plan_t& set_steps(const steps_t& steps_);
    motion_plan_t& set_position(const distance_t& position_);

    motion_plan_t& clear_motion_fragments_buffer();

    motion_plan_t& gotoxy(const distance_t& end_position_, const double& velocity_mm_s_);
    
    motion_plan_t(configuration_t& configuration);
};

}

#endif
