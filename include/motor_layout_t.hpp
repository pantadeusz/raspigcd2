#ifndef __RASPIGCD_MOTOR_LAYOUT_T_HPP__
#define __RASPIGCD_MOTOR_LAYOUT_T_HPP__

#include <distance_t.hpp>
#include <steps_t.hpp>

namespace raspigcd
{

class motor_layout_t
{
  private:
    /**
     * @brief generates instance of this object according to configuration
     */
    static motor_layout_t *generate_instance();

  public:
    /**
     * @brief converts distances in milimeters to number of ticks
     */
    virtual steps_t cartesian_to_steps(const distance_t &distances_) = 0;
    /**
     * @brief converts number of ticks to distances in milimeters
     */
    virtual distance_t steps_to_cartesian(const steps_t &steps_) = 0;

    /**
     * @brief returns instance of currently selected motor layout (from configuration)
     */
    static motor_layout_t &get();
};

} // namespace raspigcd

#endif
