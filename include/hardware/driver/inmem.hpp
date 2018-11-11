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

#ifndef __RASPIGCD_HARDWARE_DRIVER_INMEM_T_HPP__
#define __RASPIGCD_HARDWARE_DRIVER_INMEM_T_HPP__


#include <configuration.hpp>
#include <distance_t.hpp>
#include <hardware/low_buttons.hpp>
#include <hardware/low_spindles_pwm.hpp>
#include <hardware/low_steppers.hpp>
#include <hardware/stepping_commands.hpp>
#include <steps_t.hpp>

#include <functional>
#include <map>
#include <memory>
#include <thread>

namespace raspigcd {
namespace hardware {
namespace driver {

/**
 * this class can be used to fake the actual steppers driver. In case of not available hardware.
 * */

class inmem : public hardware::low_steppers
{
    std::function<void(const steps_t&)> _on_step;
public:
    int counters[RASPIGCD_HARDWARE_DOF];
    std::vector<bool> enabled;
    steps_t current_steps;

    void do_step(const single_step_command* b);
    void enable_steppers(const std::vector<bool> en);
    /**
     * @brief allows for setting callback that monitors steps execution
     */
    void set_step_callback(std::function<void(const steps_t&)> on_step_ = [](const steps_t&) {});
    inmem();
};

} // namespace driver
} // namespace hardware
} // namespace raspigcd

#endif