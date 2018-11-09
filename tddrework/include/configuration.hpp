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


#ifndef __RASPIGCD_CONFIGURATION_T_HPP__
#define __RASPIGCD_CONFIGURATION_T_HPP__

#include <string>
#include <vector>


namespace raspigcd {

namespace configuration {


enum motion_layouts {
    COREXY,
    CARTESIAN
};
/**
 * button pin and pullup setting
 * */
class button
{
public:
    int pin;
    bool pullup;
};

/**
 * configuration of spindle - spindle is controlled by separate thread
 * */
class spindle_pwm
{
public:
    int pin;
    double cycle_time_seconds;
    double duty_min;
    double duty_max;
};

/**
 * stepper motor configuration
 * */
class stepper
{
public:
    int dir;             // direction pin
    int en;              // enable pin
    int step;            // step pin
    double steps_per_mm; // steps per mm linear movement that is on this motor. This can be negative
    inline double steps_per_m() const { return steps_per_mm * 1000.0; }
    inline stepper(
        const int& _dir = 0,
        const int& _en = 0,
        const int& _step = 0,
        const double& _steps_per_mm = 0.0) : dir(_dir),
                                             en(_en),
                                             step(_step),
                                             steps_per_mm(_steps_per_mm)
    {
    }
};

class global
{
public:
    /**
 * @brief get tick duration in seconds
 * 
 * @return double tick duration in seconds
 */
    double tick_duration() const; // czas ticku w sekundach. 0.00005 = 50mikrosekund
    int tick_duration_us;         // microseconds tick time
    bool simulate_execution;      // should I use simulator by default

    std::vector<double> scale;                      ///< scale along each axis (can be negative)
    std::vector<double> max_accelerations_mm_s2;    ///<maximal acceleration on given axis (x, y, z, a) in mm/s2
    std::vector<double> max_velocity_mm_s;          ///<maximal velocity on axis in mm/s
    std::vector<double> max_no_accel_velocity_mm_s; ///<maximal velocity on axis in mm/s
    motion_layouts motion_layout;                   ///< name of layout selected: 'corexy' 'cartesian'

    std::vector<spindle_pwm> spindles;
    std::vector<stepper> steppers;
    std::vector<button> buttons;

    global& load_defaults();
    global& load(const std::string& filename);
    global& save(const std::string& filename);
};

bool operator==(const global& l, const global& r);
bool operator==(const button& l, const button& r);
bool operator==(const stepper& l, const stepper& r);
bool operator==(const spindle_pwm& l, const spindle_pwm& r);

} // namespace configuration


} // namespace raspigcd

#endif
