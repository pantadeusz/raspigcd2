#ifndef __RASPIGCD_CONFIGURATION_T_HPP__
#define __RASPIGCD_CONFIGURATION_T_HPP__

#include <vector>

namespace raspigcd
{
/**
 * configuration of spindle - spindle is controlled by separate thread
 * */
class spindle_config_t
{
  public:
    int pin;
};

/**
 * stepper motor configuration
 * */
class stepper_config_t
{
  public:
    int dir; // direction pin
    int en; // enable pin
    int step; // step pin
    double stepsPerMm; // steps per mm linear movement that is on this motor
    int direction_reverse; // should be the direction reversed?
};

/**
 * button pin
 * */
class button_config_t
{
  public:
    int pin;
    bool pullup;
};

class hardware_config_t
{
    public:
    std::vector<spindle_config_t> spindles;
    std::vector<stepper_config_t> steppers;
    std::vector<button_config_t> buttons;
};

class configuration_t
{
  public:
    /// tick time in microseconds. This is the constant value
    double tick_duration; // czas ticku w sekundach. 0.00005 = 50mikrosekund

    hardware_config_t hardware;

    /// zwraca statyczna domy
    static configuration_t &get();

    configuration_t &load_defaults();
};

} // namespace raspigcd

#endif
