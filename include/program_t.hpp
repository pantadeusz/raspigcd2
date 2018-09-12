#ifndef __RASPIGCD_PROGRAM_T_HPP__
#define __RASPIGCD_PROGRAM_T_HPP__

#include <list>
#include <string>

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
