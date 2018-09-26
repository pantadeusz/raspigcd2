#include <motor_layout_t.hpp>

#include <configuration_t.hpp>
namespace raspigcd
{

class corexy_layout_t : public motor_layout_t
{
    configuration_t *_cfg;
  public:
    std::array<double, 4> scales_; // scale along given axis
    std::array<double, 4> steps_per_milimeter_;

    steps_t cartesian_to_steps(const distance_t &distances_);
    distance_t steps_to_cartesian(const steps_t &steps_);
    corexy_layout_t(configuration_t &_cfg = configuration_t::get());
    void set_configuration(configuration_t &conf);
};

steps_t corexy_layout_t::cartesian_to_steps(const distance_t &distances_)
{
    return steps_t(
        (distances_[0] * scales_[0] + distances_[1] * scales_[1]) * steps_per_milimeter_[0], 
        (distances_[0] * scales_[0] - distances_[1] * scales_[1]) * steps_per_milimeter_[1], 
        distances_[2] * steps_per_milimeter_[2] * scales_[2], 
        distances_[3] * steps_per_milimeter_[3] * scales_[3]);
}

distance_t corexy_layout_t::steps_to_cartesian(const steps_t &steps_)
{
    return distance_t(
        0.5 * (double)(steps_[0] / steps_per_milimeter_[0] + steps_[1] / steps_per_milimeter_[1]) / scales_[0],
        0.5 * (double)(steps_[0] / steps_per_milimeter_[0] - steps_[1] / steps_per_milimeter_[1]) / scales_[1],
        steps_[2] / (steps_per_milimeter_[2] * scales_[2]),
        steps_[3] / (steps_per_milimeter_[3] * scales_[3]));
}

corexy_layout_t::corexy_layout_t(configuration_t &_cfg)
{
    set_configuration(_cfg);
}

void corexy_layout_t::set_configuration(configuration_t &conf)
{
    _cfg = &conf;
    for (int i = 0; i < DEGREES_OF_FREEDOM; i++) {
        steps_per_milimeter_[i] = _cfg->hardware.steppers.at(i).steps_per_mm;
        scales_[i] = _cfg->layout.scale[i];
    }
}


class cartesian_layout_t : public motor_layout_t
{
    configuration_t *_cfg;
  public:
    std::array<double, 4> scales_; // scale along given axis
    std::array<double, 4> steps_per_milimeter_;

    steps_t cartesian_to_steps(const distance_t &distances_);
    distance_t steps_to_cartesian(const steps_t &steps_);
    cartesian_layout_t(configuration_t &_cfg = configuration_t::get());
    void set_configuration(configuration_t &conf);
};

steps_t cartesian_layout_t::cartesian_to_steps(const distance_t &distances_)
{
    return steps_t(
        distances_[0] * steps_per_milimeter_[0] * scales_[0], 
        distances_[1] * steps_per_milimeter_[1] * scales_[1],
        distances_[2] * steps_per_milimeter_[2] * scales_[2], 
        distances_[3] * steps_per_milimeter_[3] * scales_[3]);
}

distance_t cartesian_layout_t::steps_to_cartesian(const steps_t &steps_)
{
    return distance_t(
        steps_[0] / (steps_per_milimeter_[0] * scales_[0]),
        steps_[1] / (steps_per_milimeter_[1] * scales_[1]),
        steps_[2] / (steps_per_milimeter_[2] * scales_[2]),
        steps_[3] / (steps_per_milimeter_[3] * scales_[3]));
}

cartesian_layout_t::cartesian_layout_t(configuration_t &_cfg)
{
    set_configuration(_cfg);
}
void cartesian_layout_t::set_configuration(configuration_t &conf)
{
    _cfg = &conf;
    for (int i = 0; i < DEGREES_OF_FREEDOM; i++) {
        steps_per_milimeter_[i] = _cfg->hardware.steppers.at(i).steps_per_mm;
        scales_[i] = _cfg->layout.scale[i];
    }
}



motor_layout_t *motor_layout_t::get_and_update_instance(configuration_t &conf)
{
    if (conf.layout.name == "corexy") {
        static corexy_layout_t instance_corexy(conf);
        instance_corexy.set_configuration(conf);
        return &instance_corexy;
    } else if (conf.layout.name == "cartesian") {
        static cartesian_layout_t instance_cartesian(conf);
        instance_cartesian.set_configuration(conf);
        return &instance_cartesian;
    } else {
        throw std::invalid_argument("Incorrect layout name. Only layouts allowed are: corexy cartesian");
    }
}


} // namespace raspigcd
