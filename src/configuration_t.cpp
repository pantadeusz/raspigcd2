#include "configuration_t_json.hpp"

namespace raspigcd
{

configuration_t &configuration_t::get()
{
    static configuration_t instance;
    return instance;
}



configuration_t &configuration_t::load_defaults() {
    tick_duration = 50.0/1000000.0;
    hardware.spindles = {
        {pin : 18}
    };
    hardware.steppers = {
        {
            dir : 27,
            en : 10,
            step : 22,
            stepsPerMm : 100,
            direction_reverse:0
        },
        {
            dir : 4,
            en : 10,
            step : 17,
            stepsPerMm : 100,
            direction_reverse:0
        },
        {
            dir : 9,
            en : 10,
            step : 11,
            stepsPerMm : 100,
            direction_reverse:0
        },
        {
            dir : 0,
            en : 10,
            step : 5,
            stepsPerMm : 100,
            direction_reverse:0
        }};
    hardware.buttons = {
        {pin : 21, pullup:true}, {pin : 20, pullup:true}, {pin : 16, pullup:true}, {pin : 12, pullup:true}};
    return *this;
}
/* **************************************************************************
 * CONVERSIONS
 * ************************************************************************** */


void to_json(nlohmann::json &j, const spindle_config_t &p)
{
    j = nlohmann::json{
        {"pin", p.pin}};
}

void from_json(const nlohmann::json &j, spindle_config_t &p)
{
    p.pin = j.value("pin", p.pin);
}

std::ostream &operator<<(std::ostream &os, spindle_config_t const &value)
{
    nlohmann::json j = value;
    os << j.dump(2);
    return os;
}



void to_json(nlohmann::json &j, const stepper_config_t &p)
{
    j = nlohmann::json{
        {"step", p.step},
        {"dir", p.dir},
        {"en", p.en},
        {"stepsPerMm", p.stepsPerMm}
    };
}

void from_json(const nlohmann::json &j, stepper_config_t &p)
{
    p.step = j.value("step", p.step);
    p.dir = j.value("dir", p.dir);
    p.en = j.value("en", p.en);
    p.stepsPerMm = j.value("stepsPerMm", p.stepsPerMm);
}

std::ostream &operator<<(std::ostream &os, stepper_config_t const &value)
{
    nlohmann::json j = value;
    os << j.dump(2);
    return os;
}



void to_json(nlohmann::json &j, const button_config_t &p)
{
    j = nlohmann::json{
        {"pin", p.pin},
        {"pullup", p.pullup}};
}

void from_json(const nlohmann::json &j, button_config_t &p)
{
    p.pin = j.value("pin", p.pin);
    p.pullup = j.value("pullup", p.pullup);
}

std::ostream &operator<<(std::ostream &os, button_config_t const &value)
{
    nlohmann::json j = value;
    os << j.dump(2);
    return os;
}



void to_json(nlohmann::json &j, const hardware_config_t &p)
{
    j = nlohmann::json{
        {"steppers", p.steppers},
        {"buttons", p.buttons},
        {"spindles", p.spindles}};
}

void from_json(const nlohmann::json &j, hardware_config_t &p)
{
    p.steppers = j.value("steppers", p.steppers);
    p.buttons = j.value("buttons", p.buttons);
    p.spindles = j.value("spindles", p.spindles);
}

std::ostream &operator<<(std::ostream &os, hardware_config_t const &value)
{
    nlohmann::json j = value;
    os << j.dump(2);
    return os;
}


void to_json(nlohmann::json &j, const configuration_t &p)
{
    j = nlohmann::json{
        {"hardware", p.hardware},
        {"tick_duration", p.tick_duration}};
}

void from_json(const nlohmann::json &j, configuration_t &p)
{
    p.hardware = j.value("hardware", p.hardware);
    p.tick_duration = j.value("tick_duration", p.tick_duration);
}

std::ostream &operator<<(std::ostream &os, configuration_t const &value)
{
    nlohmann::json j = value;
    os << j.dump(2);
    return os;
}

} // namespace raspigcd
