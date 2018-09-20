/*
    Raspberry Pi G-CODE interpreter
    Copyright (C) 2018  Tadeusz Puźniakowski puzniakowski.pl
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.
    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "configuration_t_json.hpp"

namespace raspigcd
{

configuration_t &configuration_t::get()
{
    static configuration_t instance;
    return instance;
}

configuration_t &configuration_t::load_defaults()
{
    tick_duration = 45.0 * 0.000001;
    hardware.spindles = {
        {pin : 18}};

    hardware.steppers = {
        stepper_config_t(27, 10, 22, 100.0, 100.0, 100.0),
        stepper_config_t(4, 10, 17, 100.0, 100, 100),
        stepper_config_t(9, 10, 11, 100.0, 100, 100),
        stepper_config_t(0, 10, 5, 100.0)};
    hardware.buttons = {
        {pin : 21, pullup : true}, {pin : 20, pullup : true}, {pin : 16, pullup : true}, {pin : 12, pullup : true}};

    //layout.name = "cartesian";
    layout.name = "corexy";
    layout.scale = {1.0, 1.0, 1.0, 1.0};
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

void to_json(nlohmann::json &j, const layout_config_t &p)
{
    j = nlohmann::json{
        {"name", p.name},
        {"scale", p.scale}};
}

void from_json(const nlohmann::json &j, layout_config_t &p)
{
    p.name = j.value("name", p.name);
    p.scale = j.value("scale", p.scale);
}

std::ostream &operator<<(std::ostream &os, layout_config_t const &value)
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
        {"steps_per_mm", p.steps_per_mm}};
}

void from_json(const nlohmann::json &j, stepper_config_t &p)
{
    p.step = j.value("step", p.step);
    p.dir = j.value("dir", p.dir);
    p.en = j.value("en", p.en);
    p.steps_per_mm = j.value("steps_per_mm", p.steps_per_mm);
    p.steps_per_mm = j.value("steps_per_m", p.steps_per_m()) / 1000.0;
    if (p.steps_per_mm <= 1.0)
        throw std::invalid_argument("the steps_per_mm must be greater than 1.0");
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
        {"layout", p.layout},
        {"tick_duration", p.tick_duration}};
}

void from_json(const nlohmann::json &j, configuration_t &p)
{
    p.hardware = j.value("hardware", p.hardware);
    p.layout = j.value("layout", p.layout);
    p.tick_duration = j.value("tick_duration", p.tick_duration);
}

std::ostream &operator<<(std::ostream &os, configuration_t const &value)
{
    nlohmann::json j = value;
    os << j.dump(2);
    return os;
}

} // namespace raspigcd
