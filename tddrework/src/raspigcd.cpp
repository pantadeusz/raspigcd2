#include <configuration.hpp>
#include <hardware_raspberry_pi.hpp>
using namespace raspigcd;
int main() {
    using namespace std::chrono_literals;

    configuration::global cfg;
    cfg.load_defaults();
    hardware::raspberry_pi_3 raspi3(cfg);
    raspi3.enable_steppers({true});
    for (int i = 0; i < 100; i++) {
        hardware::single_step_command cmnd[4];
        cmnd[3].step = 0;
        cmnd[0] = cmnd[1] = cmnd[2] = cmnd[3];
        cmnd[2].step = 1;
        cmnd[2].dir = 1;
        std::this_thread::sleep_for(1ms);
    }
    raspi3.enable_steppers({false});
    return 0;
}