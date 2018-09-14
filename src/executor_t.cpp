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

#include <executor_t.hpp>

#include <executor_pi_t.hpp>
#include <executor_sim_t.hpp>

namespace raspigcd {

executor_t &executor_t::get() {
    static executor_t *instance;
    try {
        instance = &(executor_pi_t::get());
    } catch (...) {
        instance = &(executor_sim_t::get());
    }
    return *instance;
}


} // namespace raspigcd
