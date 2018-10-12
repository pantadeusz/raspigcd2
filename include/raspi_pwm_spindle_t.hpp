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


#ifndef __raspi_pwm_spindle_t_HPP__
#define __raspi_pwm_spindle_t_HPP__

#include <configuration_t.hpp>
#include "generic_spindle_t.hpp"
#include <atomic>
#include <array>
#include <vector>
#include <memory>


namespace raspigcd {

class raspi_pwm_spindle_t : public generic_spindle_t
{
protected:
    int _pwm_pin;

    double _cycle_time_seconds; // cycle time
    double _duty_min;           // minimal signal time - represents 0 speed
    double _duty_max;           // maximal signal time - represents 1 speed (maximal)

    std::atomic<double> _duty; // current signal value

    bool _alive;

public:
    /**
     * @brief set the power fraction on spindle. The values are <-1,1>. 0 means the spindle is disabled. 1 means maximal power or speed of spindle. Some spindles can support negative values that mean reverse rotation. Not all can support that.\
     * 
     * The spindle can be also laser.
     * 
     * @param spindle power. -1 to 1 with 0 meaning spindle stopped.
     */
    void set_power(const double& pwr);
    raspi_pwm_spindle_t(const spindle_config_t &cfg_);
    // _alive
    virtual ~raspi_pwm_spindle_t()
    {
        _alive = false;
        //prevTime = prevTime + std::chrono::microseconds(200000);
    }
};


} // namespace raspigcd

#endif
