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


#ifndef __GENERIC_SPINDLE_T_HPP__
#define __GENERIC_SPINDLE_T_HPP__

#include <configuration_t.hpp>
#include <array>
#include <vector>
#include <memory>


namespace raspigcd {
/*
  there is possibility that there are multiple spindles.
*/
class generic_spindle_t
{
  protected:

  public:
    /**
     * @brief set the power fraction on spindle. The values are <-1,1>. 0 means the spindle is disabled. 1 means maximal power or speed of spindle. Some spindles can support negative values that mean reverse rotation. Not all can support that.\
     * 
     * The spindle can be also laser.
     * 
     * @param spindle power. -1 to 1 with 0 meaning spindle stopped.
     */
    virtual void set_power(const double &pwr) = 0;

    // creates spindle object based on configuration
    static std::vector<std::shared_ptr<generic_spindle_t > > get(configuration_t &cfg);

    virtual ~generic_spindle_t(){};
};

} // namespace raspigcd

#endif
