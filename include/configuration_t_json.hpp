#ifndef __RASPIGCD_CONFIGURATION_T_JSON_HPP__
#define __RASPIGCD_CONFIGURATION_T_JSON_HPP__

#include "configuration_t.hpp"
#include <json/json.hpp>

namespace raspigcd
{
//  inline bool operator==(const configuration_t &l, const configuration_t &r)
//  {
//      return l.tick_duration == r.tick_duration;
//      //return (l.step == r.step) && (l.dir == r.dir) && (l.en == r.en);
//  };


void to_json(nlohmann::json &j, const spindle_config_t &p);
void from_json(const nlohmann::json &j, spindle_config_t &p);
std::ostream &operator<<(std::ostream &os, spindle_config_t const &value);

void to_json(nlohmann::json &j, const stepper_config_t &p);
void from_json(const nlohmann::json &j, stepper_config_t &p);
std::ostream &operator<<(std::ostream &os, stepper_config_t const &value);

void to_json(nlohmann::json &j, const button_config_t &p);
void from_json(const nlohmann::json &j, button_config_t &p);
std::ostream &operator<<(std::ostream &os, button_config_t const &value);

void to_json(nlohmann::json &j, const hardware_config_t &p);
void from_json(const nlohmann::json &j, hardware_config_t &p);
std::ostream &operator<<(std::ostream &os, hardware_config_t const &value);

void to_json(nlohmann::json &j, const configuration_t &p);
void from_json(const nlohmann::json &j, configuration_t &p);
std::ostream &operator<<(std::ostream &os, configuration_t const &value);


} // namespace raspigcd

#endif
