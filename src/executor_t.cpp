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

namespace raspigcd
{

executor_t &executor_t::get(configuration_t &cfg_)
{
    static executor_t *instance;
    try
    {
        instance = &(executor_pi_t::get(cfg_));
    }
    catch (...)
    {
        instance = &(executor_sim_t::get(cfg_));
    }
    return *instance;
}

steps_t executor_t::commands_to_steps(const std::vector<executor_command_t> &commands)
{
    steps_t ret(0, 0, 0, 0);
    for (const auto &c : commands)
    {
        int dir[4] = {0, 0, 0, 0};
        // step direction
        for (auto i : {0, 1, 2, 3})
            dir[i] = c.b[i].step * (c.b[i].dir * 2 - 1);
        for (int i : {0, 1, 2, 3})
        {
            ret[i] += dir[i];
        }
    }
    return ret;
}

} // namespace raspigcd
