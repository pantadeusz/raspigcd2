#ifndef __RASPIGCD_EXECUTOR_T_HPP__
#define __RASPIGCD_EXECUTOR_T_HPP__

#include <configuration_t.hpp>

/*

 This is the lowest level executor. It generates signals to pins


 */
namespace raspigcd
{

struct executor_command_bitfield
{
    unsigned char
        step0 : 1,
        dir0 : 1,
        step1 : 1,
        dir1 : 1,
        step2 : 1,
        dir2 : 1,
        step3 : 1,
        dir3 : 1;
};

class executor_t
{
  public:
    executor_t();
    /**
   * executes list of commands.
   * 
   * if the program is already executing, then break
   * */

    int execute(const std::vector<executor_command_bitfield> &commands);
    void enable(bool en);
};

} // namespace raspigcd

#endif
