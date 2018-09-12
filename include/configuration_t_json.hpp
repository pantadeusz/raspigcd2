#ifndef __RASPIGCD_CONFIGURATION_T_JSON_HPP__
#define __RASPIGCD_CONFIGURATION_T_JSON_HPP__

#include "configuration_t.hpp"
#include <json.hpp>

namespace raspigcd
{
void to_json(nlohmann::json &j, const configuration_t &p);
void from_json(const nlohmann::json &j, configuration_t &p);
inline bool operator==(const configuration_t &l, const configuration_t &r)
{
    return l.tick_duration == r.tick_duration;
    //return (l.step == r.step) && (l.dir == r.dir) && (l.en == r.en);
};

inline std::ostream &operator<<(std::ostream &os, configuration_t const &value);

} // namespace raspigcd

#endif
