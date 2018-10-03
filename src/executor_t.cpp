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
        if (cfg_.simulate_execution) {
            std::cerr << "simulate execution.." << std::endl;
            instance = &(executor_sim_t::get(cfg_));
        } else {
            instance = &(executor_pi_t::get(cfg_));
        }
        
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
        int r = c.cmnd.repeat;
        do {
        int dir[4] = {0, 0, 0, 0};
        // step direction
        for (auto i : {0, 1, 2, 3})
            dir[i] = c.b[i].step * (c.b[i].dir * 2 - 1);
        for (int i : {0, 1, 2, 3})
        {
            ret[i] += dir[i];
        }
        } while ((r--)>0);
    }
    return ret;
}

} // namespace raspigcd
