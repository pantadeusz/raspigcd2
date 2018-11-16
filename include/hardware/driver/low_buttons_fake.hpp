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

#ifndef __RASPIGCD_HARDWARE_LOW_LEVEL_BUTTONS_FAKE_T_HPP__
#define __RASPIGCD_HARDWARE_LOW_LEVEL_BUTTONS_FAKE_T_HPP__

#include <hardware/low_buttons.hpp>

namespace raspigcd {
namespace hardware {
namespace driver {


class low_buttons_fake : public low_buttons {
private:
public:

};

}
} // namespace hardware
} // namespace raspigcd

#endif
