/*
    Raspberry Pi G-CODE interpreter
    Copyright (C) 2018  Tadeusz Puźniakowski puzniakowski.pl
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

#ifndef __RASPIGCD_CONFIGURATION_T_HPP__
#define __RASPIGCD_CONFIGURATION_T_HPP__

#include <vector>
#include <string>

namespace raspigcd
{

const int DEGREES_OF_FREEDOM = 4;


/**
 * configuration of spindle - spindle is controlled by separate thread
 * */
class spindle_config_t
{
  public:
    int pin;
};

/**
 * stepper motor configuration
 * */
class stepper_config_t
{
  public:
    int dir; // direction pin
    int en; // enable pin
    int step; // step pin
    double steps_per_mm; // steps per mm linear movement that is on this motor. This can be negative
    inline double steps_per_m() const {return steps_per_mm*1000.0;}
};

/**
 * button pin
 * */
class button_config_t
{
  public:
    int pin;
    bool pullup;
};

class hardware_config_t
{
    public:
    std::vector<spindle_config_t> spindles;
    std::vector<stepper_config_t> steppers;
    std::vector<button_config_t> buttons;
};

class layout_config_t
{
    public:
    std::vector<double> scale; ///< scale along each axis (can be negative)
    std::string name; ///< name of layout selected: 'corexy' 'cartesian'
};


class configuration_t
{
  public:
    /// tick time in microseconds. This is the constant value
    double tick_duration; // czas ticku w sekundach. 0.00005 = 50mikrosekund

    hardware_config_t hardware;
    layout_config_t layout;

    /// zwraca statyczna domy
    static configuration_t &get();

    configuration_t &load_defaults();
};

} // namespace raspigcd

#endif
