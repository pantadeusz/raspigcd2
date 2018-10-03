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

#ifndef __RASPIGCD_EXECUTOR_SIM_T_HPP__
#define __RASPIGCD_EXECUTOR_SIM_T_HPP__

#include "executor_t.hpp"
#include <steps_t.hpp>
#include <configuration_t.hpp>

#include <mutex>

/*

 This is the lowest level executor. It generates signals to pins

  this class simulates real moves

 */
namespace raspigcd
{
class executor_sim_t : public executor_t
{
    configuration_t *_cfg;
    ~executor_sim_t();
    executor_sim_t(executor_t& that) = delete;

    steps_t _steps_from_origin;
    static std::mutex _mutex_steps_from_origin;
  public:
    executor_sim_t(configuration_t &c_ = configuration_t::get());

    void set_position(const steps_t &steps);
    steps_t get_position() const;
    void terminate(){};

    /**
   * executes list of commands.
   * 
   * if the program is already executing, then break
   * */

    int execute(const std::vector<executor_command_t> &commands);
    void enable(bool en);

    static executor_sim_t &get(configuration_t &c_ = configuration_t::get());
};


} // namespace raspigcd

#endif
