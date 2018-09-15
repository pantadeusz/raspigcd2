#ifndef __RASPIGCD_EXECUTOR_T_HPP__
#define __RASPIGCD_EXECUTOR_T_HPP__

#include <configuration_t.hpp>
#include <array>
#include <steps_t.hpp>

/*

 This is the lowest level executor. It generates signals to pins


 */
namespace raspigcd
{

union executor_command_t {
    struct 
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
    } b;
    int v;
};

class executor_t
{
  protected:
    // ~executor_t();
    // executor_t();
    // executor_t(const executor_t& that) = delete; ///< singleton

  public:
    /**
     * @brief set the value of steps from origin (in steps, not in mm or other unit)
     * 
     */
    virtual void set_position(const steps_t &steps)  = 0;
    virtual steps_t get_position() const = 0;

    /**
   * executes list of commands.
   * 
   * if the program is already executing, then break
   * */

    virtual int execute(const std::vector<executor_command_t> &commands) = 0;
    virtual void enable(bool en) = 0;
    // get instance
    static executor_t & get();
};

} // namespace raspigcd

#endif
