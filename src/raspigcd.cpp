#include <program_t.hpp>
#include <configuration_t_json.hpp>
#include <vector>
#include <string>
#include <iostream>

int main(int argc, char **argv)
{
    std::vector<std::string> args(argv, argv + argc);

    auto &cfg = raspigcd::configuration_t::get();
    std::cout << cfg << std::endl;

    return 0;
}