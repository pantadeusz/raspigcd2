#include <buttons_t.hpp>
#include <raspi_buttons_t.hpp>


namespace raspigcd {

buttons_t &buttons_t::get(configuration_t &cfg_) {
    static raspi_buttons_t buttons(&cfg_);
    return buttons;
}
    

}
