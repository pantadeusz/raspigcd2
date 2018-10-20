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


#ifndef __RASPIGCD_BUTTONS_T_EXECUTOR_T_HPP__
#define __RASPIGCD_BUTTONS_T_EXECUTOR_T_HPP__

#include <configuration_t.hpp>
#include <array>
#include <steps_t.hpp>

#include <functional>
/*

 This is the lowest level executor. It generates signals to pins


 */
namespace raspigcd
{

class buttons_t
{
  protected:
    configuration_t *_cfg;
  public:
    buttons_t (configuration_t *cfg_) {
        _cfg = cfg_;
    }

    buttons_t &on_down(std::function<void(buttons_t &buttons, int button)) {
        return *this;
    }

    // get static instance
    static buttons_t &get(configuration_t &cfg_) {
        static buttons_t buttons(&cfg_);
        return buttons;
    }
};

} // namespace raspigcd

#endif
