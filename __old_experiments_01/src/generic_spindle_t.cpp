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

#include <generic_spindle_t.hpp>


#include <iostream>
#include <raspi_pwm_spindle_t.hpp>

namespace raspigcd {

class fake_pwm_spindle_t : public generic_spindle_t
{
protected:
    int _pwm_pin;

    double _cycle_time_seconds; // cycle time
    double _duty_min;           // minimal signal time - represents 0 speed
    double _duty_max;           // maximal signal time - represents 1 speed (maximal)

    std::atomic<double> _duty; // current signal value

public:
    /**
     * @brief set the power fraction on spindle. The values are <-1,1>. 0 means the spindle is disabled. 1 means maximal power or speed of spindle. Some spindles can support negative values that mean reverse rotation. Not all can support that.\
     * 
     * The spindle can be also laser.
     * 
     * @param spindle power. -1 to 1 with 0 meaning spindle stopped.
     */
    void set_power(const double& pwr)
    {
        if (pwr < 0) throw std::invalid_argument("spindle power should be 0 or more");
        if (pwr > 1.0) throw std::invalid_argument("spindle power should be less or equal 1");
        _duty = (_duty_max - _duty_min) * pwr + _duty_min;

        std::cout << "spindle power to " << pwr << "; duty: " << _duty << " :::  (" << _duty_max << " - " << _duty_min << ") *" << pwr << "+"<< _duty_min << std::endl;
    }
    fake_pwm_spindle_t(const spindle_config_t& cfg_)
    {
        _pwm_pin = cfg_.pin;
        _cycle_time_seconds = cfg_.cycle_time_seconds;
        _duty_min = cfg_.duty_min;
        _duty_max = cfg_.duty_max;

    }
    // _alive
    virtual ~fake_pwm_spindle_t()
    {
    }
};


std::vector<std::shared_ptr<generic_spindle_t>> generic_spindle_t::get(configuration_t& cfg)
{
    std::vector<std::shared_ptr<generic_spindle_t>> ret;
    for (auto& spindle_cfg_ : cfg.hardware.spindles) {
        try {
            std::shared_ptr<generic_spindle_t> raspi_pwm_spindle(new raspi_pwm_spindle_t(spindle_cfg_), [](raspi_pwm_spindle_t* ptr) {
                delete ptr;
            });
            ret.push_back(raspi_pwm_spindle);
        } catch (...) {
            std::shared_ptr<generic_spindle_t> pwm_spindle(new fake_pwm_spindle_t(spindle_cfg_), [](fake_pwm_spindle_t* ptr) {
                delete ptr;
            });
            ret.push_back(pwm_spindle);
        }
    }
    return ret;
}

} // namespace raspigcd
