#include <motor_layout_t.hpp>

#include <configuration_t.hpp>
namespace raspigcd
{

class corexy_layout_t : public motor_layout_t
{
  public:
    std::array<double, 4> scales_; // scale along given axis
    std::array<double, 4> steps_per_milimeter_;

    steps_t cartesian_to_steps(const distance_t &distances_);
    distance_t steps_to_cartesian(const steps_t &steps_);
    corexy_layout_t();
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

corexy_layout_t::corexy_layout_t()
{
    for (int i = 0; i < DEGREES_OF_FREEDOM; i++) {
        steps_per_milimeter_[i] = configuration_t::get().hardware.steppers.at(i).steps_per_mm;
        scales_[i] = configuration_t::get().layout.scale[i];
    }
}


motor_layout_t *motor_layout_t::generate_instance()
{
    static corexy_layout_t instance_corexy;
    if (configuration_t::get().layout.name == "corexy") {
        return &instance_corexy;
    } else {
        throw std::invalid_argument("Incorrect layout name. Only layouts allowed are: corexy euclidean");
    }
}


motor_layout_t &motor_layout_t::get()
{
    static motor_layout_t *instance = motor_layout_t::generate_instance();
    return *instance;
}



} // namespace raspigcd
