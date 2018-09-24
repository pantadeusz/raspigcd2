/*
    Raspberry Pi G-CODE interpreter
    Copyright (C) 2018  Tadeusz Puźniakowski puzniakowski.pl
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.
    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <chrono>
#include <configuration_t_json.hpp>
#include <distance_t.hpp>
#include <executor_t.hpp>
#include <iostream>
#include <list>
#include <motor_layout_t.hpp>
#include <string>
#include <tuple>
#include <vector>

using namespace raspigcd;
int steps_to_do(const steps_t& steps_, const steps_t& destination_steps_)
{
    int ret = 0;
    for (unsigned int i = 0; i < steps_.size(); i++) {
        ret = std::max(std::abs(steps_[i] - destination_steps_[i]), ret);
    }
    return ret;
}

/**
 * @brief generates steps to reach given destination steps
 * @arg steps_ current steps count
 * @arg destination_steps_ desired steps count
 */
std::vector<executor_command_t> chase_steps(const steps_t& steps_, steps_t destination_steps_)
{
    //auto &cfg = configuration_t::get();
    std::vector<executor_command_t> ret;
    auto steps = steps_;
    executor_command_t executor_command;
    do {
        executor_command.v = 0;
        for (unsigned int i = 0; i < steps.size(); i++) {
            executor_command.b[i].dir = ((destination_steps_[i] > steps[i]) ? 1 : 0); // TODO: (not obviosu) * ((_cfg->hardware.steppers[i].direction_reverse) ? -1 : 1);
            executor_command.b[i].step = (destination_steps_[i] - steps[i]) ? 1 : 0;
            if (destination_steps_[i] > steps[i])
                steps[i]++;
            else if (destination_steps_[i] < steps[i])
                steps[i]--;
        }
        ret.push_back(executor_command);
    } while (steps_to_do(steps, destination_steps_) > 0);
    return ret;
}

/**
 * @brief generates sinusoidal wave with given maximal amplitude in milimeters and given time in seconds
 * 
 */
std::vector<executor_command_t> generate_sin_wave_for_test(double amplitude = 15, //< in milimeters
    double T = 10,                                                                //< in seconds
    int axis = 2                                                                  //< axis to move
)
{
    auto _cfg = &(configuration_t::get());

    std::vector<executor_command_t> executor_commands;
    executor_commands.reserve((::size_t)(T / _cfg->tick_duration));
    steps_t steps;

    for (int i = 0; i < 100; i++) {
        executor_command_t e;
        e.v = 0;
        executor_commands.push_back(e);
    }

    //    steps[axis] = _cfg->hardware.steppers[axis].stepsPerMm*std::cos(0)*amplitude;
    int minimal_step_skip = 1000000000;
    for (double t = 0.0; t < T; t += _cfg->tick_duration) {
        steps_t new_steps;
        new_steps[axis] = _cfg->hardware.steppers[axis].steps_per_mm * std::cos(t * 3.141592653589793238462643 * 5) * amplitude * std::sin(3.141592653589793238462643 * t / T);
        auto st = chase_steps(steps, new_steps);
        if (st.size() == 1) {
            if (st.back().v != 0) {
                int d = 0;
                for (int i = executor_commands.size() - 1; i > 0; i--) {
                    if (executor_commands[i].v != 0)
                        break;
                    else
                        d++;
                }
                if (d < minimal_step_skip) minimal_step_skip = d;
            }
        }
        executor_commands.insert(executor_commands.end(), st.begin(), st.end());
        steps = new_steps;
    }
    std::cerr << "generated " << executor_commands.size() << "steps; minimal step skip: " << minimal_step_skip << std::endl;
    return executor_commands;
}


class motion_fragment_t
{
public:
    distance_t source, destination;
    double max_velocity;
    double source_velocity_max, destination_velocity_max;
};
class motion_plan_t
{
protected:
    std::list<motion_fragment_t> _motion_fragments;
    distance_t _current_position;

    configuration_t* _cfg;
    motor_layout_t* motor_layout;

public:
    const std::vector<executor_command_t> get_motion_plan() const
    {
        std::vector<executor_command_t> _motion_plan;


        auto move_to_position_with_given_accel = [&, this](double dt, double v, double a, double length, distance_t norm_vect) -> std::pair<std::vector<executor_command_t>, double> {
            std::vector<executor_command_t> ret;
            ret.reserve(1000000);

            steps_t steps(0, 0, 0, 0);
            distance_t position(0, 0, 0, 0);
            for (double s = 0; s < length; s += v * dt) {
                if ((v + a * dt) <= 0) {
                    std::cerr << "err: v=" << v << " <= 0; " << s << " " << length << std::endl;
                    break;
                } else {
                    v = v + a * dt;
                }
                position = position + (norm_vect * v * dt);
                auto psteps = motor_layout->cartesian_to_steps(position);
                auto st = chase_steps(steps, psteps);
                steps = psteps;
                ret.insert(ret.end(), st.begin(), st.end());
            }
            ret.shrink_to_fit();
            return {ret, v};
        };

        auto gotoxy = [&, this](const motion_fragment_t& mfrag) {
            double velocity_mm_s_ = mfrag.max_velocity;

            auto A = mfrag.source;      // start position
            auto B = mfrag.destination; // destination position

            auto diff_vect = B - A;               // difference
            double length2 = diff_vect.length2(); // length of vector in mm squared
            double length = std::sqrt(length2);   // length of vector in mm
            auto norm_vect = diff_vect / length;  // direction with length 1mm
            //double T = length/velocity_mm_s_; // time to go without accelerations
            double dt = _cfg->tick_duration; // time step
            std::vector<executor_command_t> local_motion_plan;
            local_motion_plan.reserve(100000);


            // calculate maximal acceleration
            double average_max_accel = 0;
            {
                double average_max_accel_sum = 0;
                for (int i = 0; i < 4; i++) {
                    average_max_accel += _cfg->layout.max_accelerations_mm_s2[i] * norm_vect[i];
                    average_max_accel_sum += norm_vect[i];
                }
                average_max_accel = average_max_accel / average_max_accel_sum;
            }

            double t_AM = (mfrag.max_velocity - mfrag.source_velocity_max) / average_max_accel;
            double l_AM = std::abs(mfrag.source_velocity_max * t_AM + average_max_accel * t_AM * t_AM / 2.0);

            double t_MB = (mfrag.destination_velocity_max - mfrag.max_velocity) / average_max_accel;
            double l_MB = std::abs(mfrag.source_velocity_max * t_AM + average_max_accel * t_AM * t_AM / 2.0);
            double l_M = length - (l_MB + l_AM);
            if (l_M < 0) {
                l_M = 0;
                l_MB -= std::abs(l_MB - l_AM) / 2;
                l_AM -= std::abs(l_MB - l_AM) / 2;
                if ((l_MB < 0) || (l_AM < 0)) throw std::invalid_argument("impossible accelerations");
            }

            std::cerr << "t_AM = " << t_AM << "   l_AM = " << l_AM << std::endl;
            std::cerr << "t_MB = " << t_MB << "   l_MB = " << l_MB << std::endl;
            std::cerr << "l_M = " << l_M << std::endl;
            double temporary_velocity = mfrag.source_velocity_max;
            if (l_AM > 0) {
                std::cerr << "move_to_position_with_given_accel(" << dt << ", " << mfrag.source_velocity_max << ", " << average_max_accel << ", " << l_AM << ", norm_vect)" << std::endl;
                auto st_accel = move_to_position_with_given_accel(dt, mfrag.source_velocity_max, average_max_accel, l_AM, norm_vect);
                local_motion_plan.insert(local_motion_plan.end(), st_accel.first.begin(), st_accel.first.end());
                temporary_velocity = st_accel.second;
            }
            if (l_M > 0) {
                std::cerr << "move_to_position_with_given_accel(" << dt << "/*  */, " << temporary_velocity << ", " << 0 << ", " << l_M << ", norm_vect)" << std::endl;
                auto st_const = move_to_position_with_given_accel(dt, temporary_velocity, 0, l_M, norm_vect);
                local_motion_plan.insert(local_motion_plan.end(), st_const.first.begin(), st_const.first.end());
                temporary_velocity = st_const.second;
            }
            if (l_MB > 0) {
                std::cerr << "move_to_position_with_given_accel(" << dt << ", " << temporary_velocity << ", " << (-average_max_accel) << ", " << l_MB << ", norm_vect)" << std::endl;
                auto st_decel = move_to_position_with_given_accel(dt, temporary_velocity, -average_max_accel, l_MB, norm_vect);
                local_motion_plan.insert(local_motion_plan.end(), st_decel.first.begin(), st_decel.first.end());
                temporary_velocity = st_decel.second;
            }
            auto end_position_actual = executor_t::commands_to_steps(local_motion_plan);
            end_position_actual = end_position_actual + motor_layout->cartesian_to_steps(mfrag.source);
            //auto end_position_wanted = motor_layout->cartesian_to_steps(mfrag.destination);
            auto direction_to_do = mfrag.destination - motor_layout->steps_to_cartesian(end_position_actual);
            auto direction_to_do_length = std::sqrt(direction_to_do.length2());
            if (direction_to_do_length > 0) {
                std::cerr << "move_to_position_with_given_accel(" << dt << ", " << temporary_velocity << ", " << 0 << ", " << direction_to_do_length << ", norm_vect)" << std::endl;
                auto st_fix = move_to_position_with_given_accel(dt, temporary_velocity, 0, direction_to_do_length, direction_to_do / direction_to_do_length);
                local_motion_plan.insert(local_motion_plan.end(), st_fix.first.begin(), st_fix.first.end());
                temporary_velocity = st_fix.second;
            }
            local_motion_plan.shrink_to_fit();
            _motion_plan.insert(_motion_plan.end(), local_motion_plan.begin(), local_motion_plan.end());
            return executor_t::commands_to_steps(local_motion_plan);
        };

        steps_t steps = motor_layout->cartesian_to_steps(_motion_fragments.front().source);
        for (auto& mfrag : _motion_fragments) {
            auto steps_diff = gotoxy(mfrag); //.destination, mfrag.max_velocity);
            steps = steps + steps_diff;
            auto st = chase_steps(steps, motor_layout->cartesian_to_steps(mfrag.destination));
            _motion_plan.insert(_motion_plan.end(), st.begin(), st.end());
        }
        return _motion_plan;
    };
    const steps_t get_steps() const { return motor_layout->cartesian_to_steps(_current_position); };
    const distance_t get_position() const { return _current_position; };

    motion_plan_t& set_steps(const steps_t& steps_)
    {
        _current_position = motor_layout->steps_to_cartesian(steps_);
        return *this;
    };
    motion_plan_t& set_position(const distance_t& position_)
    {
        _current_position = position_;
        return *this;
    };
    // motion_plan_t &set_motion_plan(const std::vector<executor_command_t>& mp_) {_motion_plan = mp_; return *this;};

    motion_plan_t& gotoxy(const distance_t& end_position_, const double& velocity_mm_s_)
    {
        _motion_fragments.push_back({
            _current_position,
            end_position_,
            max_velocity : velocity_mm_s_,
            source_velocity_max : 0.001,
            destination_velocity_max : 0.001
        });
        _current_position = end_position_;
        return *this;
    }

    /**
     * @brief calculates maximal acceleration on plan in mm/s2
     * 
     */
    std::vector<std::array<int, DEGREES_OF_FREEDOM>> get_AMerations(int tticks_count = 500)
    {
        auto _motion_plan = get_motion_plan();

        std::vector<std::array<int, DEGREES_OF_FREEDOM>> velocities(_motion_plan.size());
        int ticks_l[DEGREES_OF_FREEDOM] = {0, 0, 0, 0};
        int ticks_r[DEGREES_OF_FREEDOM] = {0, 0, 0, 0};
        for (int dof = 0; dof < DEGREES_OF_FREEDOM; dof++) {
            for (int i = 1; i <= std::min((int)tticks_count >> 1, (int)_motion_plan.size() - 1); i++) {
                ticks_r[dof] += ((int)_motion_plan[i].b[dof].step) * (((int)(_motion_plan[i].b[dof].dir << 1)) - 1);
            }
        }
        for (int i = 0; i < (int)_motion_plan.size(); i++) {
            for (int dof = 0; dof < DEGREES_OF_FREEDOM; dof++) {
                if (_motion_plan[i].b[dof].step) velocities[i][dof] = ticks_r[dof] - ticks_l[dof];
            }
            for (int dof = 0; dof < DEGREES_OF_FREEDOM; dof++) {
                {
                    auto ticks_r_i = (int)(i + 1 + (tticks_count >> 1));
                    if (ticks_r_i < (int)_motion_plan.size())
                        ticks_r[dof] += ((int)_motion_plan[ticks_r_i].b[dof].step) * (((int)(_motion_plan[ticks_r_i].b[dof].dir << 1)) - 1);
                    if (i < (int)_motion_plan.size()) ticks_r[dof] -= ((int)_motion_plan[i + 1].b[dof].step) * (((int)(_motion_plan[i + 1].b[dof].dir << 1)) - 1);
                }
                {
                    auto ticks_l_i = (int)(i - (tticks_count >> 1));
                    if (ticks_l_i >= 0)
                        ticks_l[dof] -= ((int)_motion_plan[ticks_l_i].b[dof].step) * (((int)(_motion_plan[ticks_l_i].b[dof].dir << 1)) - 1);
                    //if (i > 0 )
                    ticks_l[dof] += ((int)_motion_plan[i].b[dof].step) * (((int)(_motion_plan[i].b[dof].dir << 1)) - 1);
                }
            }
        }
        return velocities;
    }

    motion_plan_t(configuration_t& configuration = configuration_t::get())
    {
        motor_layout = motor_layout_t::get_and_update_instance(configuration);
        _cfg = &configuration;
    }
};

int main(int argc, char** argv)
{
    std::vector<std::string> args(argv, argv + argc);

    static auto& cfg = configuration_t::get().load_defaults();
    std::cout << cfg << std::endl;
    executor_t& executor = executor_t::get(cfg);

    executor.enable(true);
    //executor.execute(executor_commands);
    //executor.execute(generate_sin_wave_for_test());
    {
        motion_plan_t mp(cfg);
        mp.gotoxy(distance_t{5.0, 0.0, 0.0, 0.0}, 20.0)
            .gotoxy(distance_t{5.0, -5.0, 0.0, 0.0}, 20.0)
            .gotoxy(distance_t{0.0, -5.0, 0.0, 0.0}, 20.0)
            .gotoxy(distance_t{0.0, 0.0, 0.0, 0.0}, 20.0);
        ///  mp.gotoxy(distance_t{0.0,0.0,5.0,0.0},10.0)
        ///      .gotoxy(distance_t{0.0,0.0,-5.0,0.0},10.0)
        ///      .gotoxy(distance_t{0.0,0.0,0.0,0.0},10.0);

        //{
        //    auto t0 = std::chrono::system_clock::now();
        //            mp.fix_accelerations(300);
        //    auto t1 = std::chrono::system_clock::now();
        //    double elaspedTimeMs = std::chrono::duration<double>(t1-t0).count();
        //    std::cerr << "fix_accelerations time: " << elaspedTimeMs << " seconds"<< std::endl;
        //}
        //
        //auto acc = mp.get_AMerations();
        //std::cerr << "accelerations fixed to " << mp.get_motion_plan().size() << std::endl;
        //int i = 0;
        //for(auto e: acc) {
        //    std::cout << "dof" << i;
        //    for (auto v : e) std::cout << " " << v;
        //    std::cout  << std::endl;
        //    i++;
        //}

        executor.execute(mp.get_motion_plan());
    }
    executor.enable(false);

    return 0;
}
