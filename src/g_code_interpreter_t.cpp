#include "g_code_interpreter_t.hpp"


namespace raspigcd {
// executes gcode command
std::string g_code_interpreter_t::g(const std::string& gcd, std::function<std::string(const char, const std::map<char, double>&)> callback_)
{
    std::string result;
    //        line_++;
    int i = 0;

    auto skipWhitespaces = [&]() {
        while ((i < (int)gcd.length()) && ((gcd[i] == ' ') || (gcd[i] == '\n') || (gcd[i] == '\r')))
            i++;
    };

    if ((gcd.length() == 0) || (gcd[0] == ';')) {
        result = "ok comment";
    } else {
        std::vector<std::pair<char, std::string>> commandSequence;
        char cmndCode = ' ';

        while (i < (int)gcd.length()) {
            skipWhitespaces();
            if (i < (int)gcd.length()) {
                if ((gcd[i] >= 'A') && (gcd[i] <= 'Z')) {
                    cmndCode = gcd[i];
                    i++;
                } else {
                    result = "err bad gcode command";
                    i = gcd.length();
                    break;
                }
            }
            skipWhitespaces();

            std::string codeNumS;
            while ((i < (int)gcd.length()) && (((gcd[i] >= '0') && (gcd[i] <= '9')) || (gcd[i] == '-') || (gcd[i] == '.'))) {
                codeNumS = codeNumS + gcd[i];
                i++;
            }

            //std::pair < std::string, std::string > commandPartPair = std::make_pair(cmndCode,codeNumS);
            //std::cout << "K:" << cmndCode << " : " << codeNumS << std::endl;

            commandSequence.push_back(std::make_pair(cmndCode, codeNumS));
        }

        std::vector<std::map<char, double>> subCommandsV;
        std::vector<char> subCommandsN;
        int subCommandsIdx = -1;
        for (auto cmnd : commandSequence) {
            if (subCommandsIdx >= 0)
                subCommandsV[subCommandsIdx][cmnd.first] = std::stod(cmnd.second);
            if (_gcode_commands.count(cmnd.first)) {
                std::map<char, double> m = {{cmnd.first, std::stod(cmnd.second)}};
                subCommandsIdx++;
                subCommandsN.push_back(cmnd.first);
                subCommandsV.push_back(m);
            }
        }

        for (int i = 0; i <= subCommandsIdx; i++) {
            //result = result + ((i > 0) ? "; " : "") + _gcode_commands[subCommandsN[i]](subCommandsV[i]);
            result = result + ((i > 0) ? "; " : "") + callback_(subCommandsN[i], subCommandsV[i]);
        }
    }
    return result;
}


std::list<std::string> g_code_interpreter_t::execute_gcode_lines(std::list<std::string> lines)
{
    std::list<std::string> ret;
    std::pair<char, double> previous_code = {'_', 0.0};
    for (auto& line : lines) {
        g(line, [&, this](const char code_, const std::map<char, double>& params_) {
            std::pair<char, double> current_code = {code_, params_.at(code_)};
            std::cout << "kod: " << code_ << std::endl;
            for (auto e : params_) {
                std::cout << "  " << e.first << " " << e.second << std::endl;
            }
            if ((current_code.first != 'G') || (current_code.second > 1)) {
                //std::cout << "break and execute code" << std::endl;
                auto mp = _motion_planner->get_motion_plan();
                _executor->execute(mp);
                if (mp.size() > 0) _motion_planner->clear_motion_fragments_buffer();
            }
            //now execute code..
            _gcode_commands[code_](params_);
            previous_code = current_code;

            ret.push_back("ok ...");
            return "ok";
        });
    }
    return ret;
}

std::list<std::string> g_code_interpreter_t::execute_gcode_string(const std::string gcode_text_)
{
    std::list<std::string> lines;
    std::istringstream ss(gcode_text_);
    std::string g;
    while (std::getline(ss, g, '\n')) {
        lines.push_back(g);
    }
    return execute_gcode_lines(lines);
}
g_code_interpreter_t::g_code_interpreter_t(configuration_t* cfg, executor_t* executor, motion_plan_t* motion_planner_)
{
    _cfg = cfg;
    _executor = executor;
    _motion_planner = motion_planner_;

    _fr_multiplier = 1;                             // 1 - mm/s   60 - mm/m
    _gspeed[0] = _cfg->layout.max_velocity_mm_s[0]; //
    for (auto& e : _cfg->layout.max_velocity_mm_s)
        _gspeed[0] = std::max(e, _gspeed[0]);
    _gspeed[1] = _cfg->layout.max_no_accel_velocity_mm_s;

    _gcode_commands['G'] = [this](const std::map<char, double>& m) -> std::string {
        distance_t p;
        if (((int)m.at('G') == 0) || ((int)m.at('G') == 1)) { // work move
            int coordsToMove = 0;
            coordsToMove += m.count('X') * 1;
            coordsToMove += m.count('Y') * 2;
            coordsToMove += m.count('Z') * 4;
            p = _motion_planner->get_position();
            distance_t pos(m.count('X') ? m.at('X') : p[0],
                m.count('Y') ? m.at('Y') : p[1],
                m.count('Z') ? m.at('Z') : p[2],
                m.count('A') ? m.at('A') : p[3]);

            auto frate = _gspeed[(int)m.at('G')];
            if ((int)m.at('G') == 0) {
                if (m.count('F') == 0) {
                    frate = _gspeed[(int)m.at('G')] * _fr_multiplier;
                } else {
                    _gspeed[(int)m.at('G')] = frate / _fr_multiplier;
                }
            }
            _motion_planner->gotoxy(pos, _gspeed[(int)m.at('G')]);

            // machine_.get()->gotoXYZ(pos, m.at('F') / frMultiplier, coordsToMove, G0speedV0, G0speedV0ddt); // fast movement
            return "ok executed movement";
        } else if ((int)m.at('G') == 4) {
            int t = 0; // in miliseconds
            if (m.count('P') == 1) {
                t = m.at('P');
            } else if (m.count('X') == 1) {
                t = 1000 * m.at('X');
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(t));
            return "ok dwell executed";
        } else if ((int)m.at('G') == 92) {
            // G92 -- reset coordinates
            p[0] = (m.count('X') == 0) ? 0.0 : m.at('X');
            p[0] = (m.count('Y') == 0) ? 0.0 : m.at('Y');
            p[0] = (m.count('Z') == 0) ? 0.0 : m.at('Z');

            _motion_planner->set_position(p);
            _executor->set_position(_motion_planner->get_steps());
            //                machine_.get()->setPosition(Position(m.at('X'), m.at('Y'), m.at('Z')));
            return "ok reset position to X:" + std::to_string(p[0]) + " Y:" + std::to_string(p[1]) + " Z:" + std::to_string(p[2]) + "";

            /*} else if ((int)m.at('G') == 28) {
                finish();
                // G28 -- move to origin
                
                p = _motion_planner->get_position();
                return "ok origin X:" + std::to_string(p[0]) + " Y:" + std::to_string(p[1]) + " Z:" + std::to_string(p[2]) + "";

                // G92 -- reset coordinates */
        } else
            return "!! unsupported G code number";
    };
    _gcode_commands['M'] = [this](const std::map<char, double>& m) -> std::string {
        distance_t p;
        //std::array<unsigned char, 4> endstops;
        switch ((int)m.at('M')) {
        /* case 3:
                machine_.get()->spindleEnabled(true);
                t = 7000;
                if (m.count('P') == 1) {
                    t = m.at('P');
                } else if (m.count('X') == 1) {
                    t = 1000 * m.at('X');
                }
                if (t > 0)
                    machine_.get()->pause(t);

                return "ok spindle on CW";
            case 5:
                machine_.get()->spindleEnabled(false);
                t = 0;
                if (m.count('P') == 1) {
                    t = m.at('P');
                } else if (m.count('X') == 1) {
                    t = 1000 * m.at('X');
                }
                if (t > 0)
                    machine_.get()->pause(t);
                return "ok spindle off"; */
        case 17:
            _executor->enable(true);
            return "ok steppers on";
        case 18:
            _executor->enable(false);
            return "ok steppers off";
        case 114:
            p = _motion_planner->get_position();
            return "ok position X:" + std::to_string(p[0]) + " Y:" + std::to_string(p[1]) + " Z:" + std::to_string(p[2]) + "";
            /* case 119:
                endstops = machine_.get()->getEndstops();
                return "ok endstops: X:" + std::to_string(endstops[0]) + " Y:" + std::to_string(endstops[1]) + " Z:" + std::to_string(endstops[2]) + " T:" + std::to_string(endstops[3]) + "";
            case 577:
                int btnn = machine_.get()->waitForEndstopTrigger();
                return "ok button pressed " + std::to_string(btnn); */
        }
        return "!! unrecognized command";
    };
}


} // namespace raspigcd
