#include <configuration.hpp>
#include <hardware_raspberry_pi.hpp>
using namespace raspigcd;
int main() {
    using namespace std::chrono_literals;

    configuration::global cfg;
    hardware::raspberry_pi_3 raspi3(cfg);
    raspi3.enable_steppers({true});
    std::this_thread::sleep_for(5s);
    raspi3.enable_steppers({false});
    return 0;
}