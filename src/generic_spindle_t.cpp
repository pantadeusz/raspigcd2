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

namespace raspigcd {
std::vector<std::shared_ptr<generic_spindle_t>> generic_spindle_t::get(configuration_t& cfg)
{
    std::vector<std::shared_ptr<generic_spindle_t>> ret;
    return ret;
}

} // namespace raspigcd
