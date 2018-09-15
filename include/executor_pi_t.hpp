#ifndef __RASPIGCD_EXECUTOR_PI_T_HPP__
#define __RASPIGCD_EXECUTOR_PI_T_HPP__

#include <executor_t.hpp>
#include <configuration_t.hpp>
#include <steps_t.hpp>

/*

 This is the lowest level executor. It generates signals to pins


 */
namespace raspigcd
{
class executor_pi_t : public executor_t
{
    ~executor_pi_t();
    executor_pi_t();
    executor_pi_t(const executor_t& that) = delete;
  public:
    steps_t steps_from_origin_;


    void set_position(const steps_t &steps);
    steps_t get_position() const;

    /**
   * executes list of commands.
   * 
   * if the program is already executing, then break
   * */

    int execute(const std::vector<executor_command_t> &commands);
    void enable(bool en);

    static executor_pi_t &get();
};


} // namespace raspigcd

#endif
