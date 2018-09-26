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
    int dir;                      // direction pin
    int en;                       // enable pin
    int step;                     // step pin
    double steps_per_mm;          // steps per mm linear movement that is on this motor. This can be negative
    double max_velocity;          // maximal velocity in mm/s
    inline double steps_per_m() const { return steps_per_mm * 1000.0; }
    inline stepper_config_t(
        const int &_dir = 0,
        const int &_en = 0,
        const int &_step = 0,
        const double &_steps_per_mm = 0.0,
        const double &_max_velocity = 0.0) : dir(_dir),
                                                en(_en),
                                                step(_step),
                                                steps_per_mm(_steps_per_mm),
                                                max_velocity(_max_velocity)
    {
    }
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
    std::vector<double> max_accelerations_mm_s2; ///<maximal acceleration on given axis (x, y, z, a) in mm/s2
    std::vector<double> max_velocity_mm_s; ///<maximal velocity on axis in mm/s
    double max_no_accel_velocity_mm_s; ///<maximal velocity on axis in mm/s
    
    std::string name;          ///< name of layout selected: 'corexy' 'cartesian'
};

class configuration_t
{
  public:
    /// tick time in microseconds. This is the constant value
    double tick_duration; // czas ticku w sekundach. 0.00005 = 50mikrosekund

    hardware_config_t hardware;
    layout_config_t layout;

    bool simulate_execution; // should I use simulator by default

    /// zwraca statyczna domy
    static configuration_t &get();

    configuration_t &load_defaults();
};

} // namespace raspigcd

#endif
