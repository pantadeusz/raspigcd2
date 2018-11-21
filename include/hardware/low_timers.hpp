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

#ifndef __RASPIGCD_HARDWARE_LOW_LEVEL__TIMERS_T_HPP__
#define __RASPIGCD_HARDWARE_LOW_LEVEL__TIMERS_T_HPP__

#include <chrono>

namespace raspigcd {
namespace hardware {


class low_timers
{
private:
public:
    /**
     * @brief delay in seconds. This can be fraction of a second
     * 
     * @param t 
     */
    virtual void wait_s(const double t) = 0;

    /**
     * @brief start the timer
     * 
     */
    virtual std::chrono::high_resolution_clock::time_point start_timing() = 0;

    /**
     * @brief wait for the tick to end.
     * Remember to run start_timing first!
     */
    virtual std::chrono::high_resolution_clock::time_point wait_for_tick_s(const std::chrono::high_resolution_clock::time_point &prev_timer, const double t) = 0;
};

} // namespace hardware
} // namespace raspigcd

#endif
