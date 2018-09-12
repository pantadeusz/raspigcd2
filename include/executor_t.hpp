#ifndef __RASPIGCD_EXECUTOR_T_HPP__
#define __RASPIGCD_EXECUTOR_T_HPP__

#include <configuration_t.hpp>

/*

 This is the lowest level executor. It generates signals to pins


 */
namespace raspigcd
{

class executor_t
{
  private:
    executor_t();
    ~executor_t();
  public:
  /**
   * executes list of commands.
   * 
   * if the program is already executing, then break
   * */
    void execute();
    void terminate();
    void pause();
    void status();
    static executor_t &get();
};

} // namespace raspigcd

#endif
