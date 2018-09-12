#ifndef __RASPIGCD_PROGRAM_T_JSON_HPP__
#define __RASPIGCD_PROGRAM_T_JSON_HPP__

#include <list>
#include <string>
#include <json/json.hpp>

namespace raspigcd
{

class program_t
{
  public:
    std::list<std::string> commands;
    std::list<std::string> result;
};

} // namespace raspigcd

#endif
