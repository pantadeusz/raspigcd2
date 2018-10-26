#include <configuration.hpp>
#include <json/json.hpp>

namespace raspigcd {
namespace configuration {

/* **************************************************************************
 * CONVERSIONS
 * ************************************************************************** */

void to_json(nlohmann::json& j, const global& p);
void from_json(const nlohmann::json& j, global& p);
std::ostream& operator<<(std::ostream& os, global const& value);


void to_json(nlohmann::json& j, const spindle_pwm& p);
void from_json(const nlohmann::json& j, spindle_pwm& p);
std::ostream& operator<<(std::ostream& os, spindle_pwm const& value);
void to_json(nlohmann::json& j, const stepper& p);
void from_json(const nlohmann::json& j, stepper& p);
std::ostream& operator<<(std::ostream& os, stepper const& value);

void to_json(nlohmann::json& j, const button& p);

void from_json(const nlohmann::json& j, button& p);

} // namespace configuration
} // namespace raspigcd