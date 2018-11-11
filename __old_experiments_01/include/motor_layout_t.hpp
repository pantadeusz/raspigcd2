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

#ifndef __RASPIGCD_MOTOR_LAYOUT_T_HPP__
#define __RASPIGCD_MOTOR_LAYOUT_T_HPP__

#include <distance_t.hpp>
#include <steps_t.hpp>
#include <configuration_t.hpp>

namespace raspigcd
{

class motor_layout_t
{
  private:

  public:
    /**
     * @brief converts distances in milimeters to number of ticks
     */
    virtual steps_t cartesian_to_steps(const distance_t &distances_) = 0;
    /**
     * @brief converts number of ticks to distances in milimeters
     */
    virtual distance_t steps_to_cartesian(const steps_t &steps_) = 0;

    /**
     * @brief generates instance of this object according to configuration
     */
    static motor_layout_t *get_and_update_instance(configuration_t &conf);
};

} // namespace raspigcd

#endif