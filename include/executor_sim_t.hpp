#ifndef __RASPIGCD_EXECUTOR_SIM_T_HPP__
#define __RASPIGCD_EXECUTOR_SIM_T_HPP__

#include <executor_t.hpp>
#include <steps_t.hpp>
#include <configuration_t.hpp>

/*

 This is the lowest level executor. It generates signals to pins

  this class simulates real moves

 */
namespace raspigcd
{
class executor_sim_t : public executor_t
{
    ~executor_sim_t();
    executor_sim_t();
    executor_sim_t(const executor_t& that) = delete;

    steps_t steps_from_origin_;

  public:

    void set_position(const steps_t &steps);
    steps_t get_position() const;


    /**
   * executes list of commands.
   * 
   * if the program is already executing, then break
   * */

    int execute(const std::vector<executor_command_t> &commands);
    void enable(bool en);

    static executor_sim_t &get();
};


} // namespace raspigcd

#endif
