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

#include <executor_t.hpp>
#include <configuration_t_json.hpp>
#include <distance_t.hpp>
#include <motor_layout_t.hpp>
#include <vector>
#include <list>
#include <string>
#include <iostream>
#include <chrono>

using namespace raspigcd;
int steps_to_do(const steps_t &steps_, const steps_t &destination_steps_)
{
    int ret = 0;
    for (unsigned int i = 0; i < steps_.size(); i++)
    {
        ret = std::max(std::abs(steps_[i] - destination_steps_[i]), ret);
    }
    return ret;
}

/**
 * @brief generates steps to reach given destination steps
 * @arg steps_ current steps count
 * @arg destination_steps_ desired steps count
 */
std::vector<executor_command_t> chase_steps(const steps_t &steps_, steps_t destination_steps_)
{
    //auto &cfg = configuration_t::get();
    std::vector<executor_command_t> ret;
    auto steps = steps_;
    executor_command_t executor_command;
    do
    {
        executor_command.v = 0;
        for (unsigned int i = 0; i < steps.size(); i++)
        {
            executor_command.b[i].dir = ((destination_steps_[i] > steps[i]) ? 1 : 0);// TODO: (not obviosu) * ((_cfg->hardware.steppers[i].direction_reverse) ? -1 : 1);
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
                                           double T = 10,         //< in seconds
                                           int axis = 2           //< axis to move
                                           )
{
    auto _cfg = &(configuration_t::get());

    std::vector<executor_command_t> executor_commands;
    executor_commands.reserve((::size_t)(T/_cfg->tick_duration));
    steps_t steps;

    for (int i = 0; i < 100; i++)
    {
        executor_command_t e;
        e.v = 0;
        executor_commands.push_back(e);
    }

    //    steps[axis] = _cfg->hardware.steppers[axis].stepsPerMm*std::cos(0)*amplitude;
    int minimal_step_skip = 1000000000;
    for (double t = 0.0; t < T; t += _cfg->tick_duration)
    {
        steps_t new_steps;
        new_steps[axis] = _cfg->hardware.steppers[axis].steps_per_mm * std::cos(t * 3.141592653589793238462643 * 5) * amplitude * std::sin(3.141592653589793238462643 * t / T);
        auto st = chase_steps(steps, new_steps);
        if (st.size() == 1) {
            if (st.back().v != 0){
                int d = 0;
                for (int i = executor_commands.size() -1; i > 0; i--) {
                     if (executor_commands[i].v != 0) break;
                     else d++;
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




class motion_fragment_t {
public:
    distance_t source, destination;
    double max_velocity;
    double source_velocity_max, destination_velocity_max;
};
class motion_plan_t {
protected:
    std::list<motion_fragment_t> _motion_fragments;
    distance_t _current_position;

    configuration_t *_cfg;
    motor_layout_t *motor_layout;
public:
    const std::vector<executor_command_t> get_motion_plan() const {
        std::vector<executor_command_t> _motion_plan;


        auto move_to_position_with_given_accel = [&,this](double dt, double v, double a, double length, distance_t norm_vect) -> std::vector<executor_command_t> {
            std::vector<executor_command_t> ret;
            ret.reserve(1000000);
            steps_t steps(0,0,0,0);
            distance_t position(0,0,0,0);
            for (double s = 0; s < length; s+= v*dt) {
                v = v + a * dt;
                position = position + (norm_vect*v * dt);
                auto psteps = motor_layout->cartesian_to_steps(position);
                auto st = chase_steps(steps, psteps);
                steps = psteps;
                ret.insert(ret.end(), st.begin(), st.end());
            }
            ret.shrink_to_fit();
            return ret;
        };

        auto gotoxy = [&,this](const motion_fragment_t & mfrag) {
            double velocity_mm_s_ = mfrag.max_velocity;

            auto A = mfrag.source; // start position
            auto B = mfrag.destination; // destination position

            auto diff_vect = B-A; // difference
            double length2 = diff_vect.length2(); // length of vector in mm squared
            double length = std::sqrt(length2); // length of vector in mm
            auto norm_vect = diff_vect/length; // direction with length 1mm
            //double T = length/velocity_mm_s_; // time to go without accelerations
            double dt = _cfg->tick_duration; // time step
            double ds = velocity_mm_s_*dt; // TODO: distance step - this is going to be variable in futute
            _motion_plan.reserve(_motion_plan.size()+100000);

            std::vector<steps_t> from_a_to_b; // series of steps from A to B (absolute positions)
            std::vector<steps_t> from_b_to_a;
            steps_t start_steps = motor_layout->cartesian_to_steps(A); // current steps value
            steps_t end_steps = motor_layout->cartesian_to_steps(B); // where are we going

            /*
            double s = ds;
            for (int i = 1; s <= length; i++) {
                auto p = A + norm_vect*s;
                auto psteps = motor_layout->cartesian_to_steps(p);
                from_a_to_b.push_back(psteps);
                s += ds;
            }

            if (from_a_to_b.back() != end_steps) {
                std::cerr << "wrong end steps " << std::endl;
                from_a_to_b.push_back(end_steps);
            }

            auto pos0 = start_steps;
            for (auto &p: from_a_to_b) {
                auto st = chase_steps(pos0, p);
                _motion_plan.insert(_motion_plan.end(), st.begin(), st.end());
                pos0=p;
            }
            */
           std::cerr << "move_to_position_with_given_accel("<<dt<<", "<<ds<<", " << 0<< ", "<< length<<", norm_vect)" << std::endl;
           auto st = move_to_position_with_given_accel(dt, velocity_mm_s_, 0, length, norm_vect);
           std::cerr << "move_to_position_with_given_accel("<<dt<<", "<<ds<<", " << 0<< ", "<< length<<", norm_vect) OK" << std::endl;

            _motion_plan.insert(_motion_plan.end(), st.begin(), st.end());
            _motion_plan.shrink_to_fit();
            return executor_t::commands_to_steps(st);
        };

        steps_t steps = motor_layout->cartesian_to_steps(_motion_fragments.front().source);
        for (auto &mfrag: _motion_fragments) {
            auto steps_diff = gotoxy(mfrag);//.destination, mfrag.max_velocity);
            steps = steps + steps_diff;
            auto st = chase_steps(steps, motor_layout->cartesian_to_steps(mfrag.destination));
            _motion_plan.insert(_motion_plan.end(), st.begin(), st.end());
        }
        return _motion_plan;
    };
    const steps_t get_steps() const {return motor_layout->cartesian_to_steps(_current_position);};
    const distance_t get_position() const {return _current_position;};

    motion_plan_t &set_steps(const steps_t &steps_) {_current_position = motor_layout->steps_to_cartesian(steps_);return *this;};
    motion_plan_t &set_position(const distance_t &position_) {_current_position = position_;return *this;};
    // motion_plan_t &set_motion_plan(const std::vector<executor_command_t>& mp_) {_motion_plan = mp_; return *this;};

    motion_plan_t &gotoxy(const distance_t &end_position_, const double &velocity_mm_s_) {
        _motion_fragments.push_back({
            _current_position, end_position_,
            max_velocity:velocity_mm_s_,
            source_velocity_max:0.0, destination_velocity_max:0.0
        });
        _current_position = end_position_;
        return *this;
    }

    /**
     * @brief calculates maximal acceleration on plan in mm/s2
     * 
     */
    std::vector < std::array<int, DEGREES_OF_FREEDOM> > get_accelerations(int tticks_count = 500)
    {
        auto _motion_plan = get_motion_plan();

        std::vector < std::array<int, DEGREES_OF_FREEDOM> > velocities(_motion_plan.size());
        int ticks_l[DEGREES_OF_FREEDOM] = {0,0,0,0};
        int ticks_r[DEGREES_OF_FREEDOM] = {0,0,0,0};
        for (int dof = 0; dof < DEGREES_OF_FREEDOM; dof++) {
            for (int i = 1; i <= std::min((int)tticks_count>>1,(int)_motion_plan.size()-1); i++) {
                ticks_r[dof] += ((int)_motion_plan[i].b[dof].step) * (((int)(_motion_plan[i].b[dof].dir << 1)) - 1);
            }
        }
        for (int i = 0; i < (int)_motion_plan.size(); i++)
        {
            for (int dof = 0; dof < DEGREES_OF_FREEDOM; dof++) {
                if (_motion_plan[i].b[dof].step) velocities[i][dof] = ticks_r[dof] - ticks_l[dof];
            }
            for (int dof = 0; dof < DEGREES_OF_FREEDOM; dof++) {
                {
                    auto ticks_r_i = (int)(i+1 + (tticks_count >> 1));
                    if (ticks_r_i < (int)_motion_plan.size()) 
                        ticks_r[dof] += ((int)_motion_plan[ticks_r_i].b[dof].step) * (((int)(_motion_plan[ticks_r_i].b[dof].dir << 1)) - 1);
                    if (i <  (int)_motion_plan.size()) ticks_r[dof] -= ((int)_motion_plan[i+1].b[dof].step) * (((int)(_motion_plan[i+1].b[dof].dir << 1)) - 1);
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

    motion_plan_t(configuration_t &configuration = configuration_t::get()) {
        motor_layout = motor_layout_t::get_and_update_instance(configuration);
        _cfg = &configuration;
    }
};

int main(int argc, char **argv)
{
    std::vector<std::string> args(argv, argv + argc);

    static auto &cfg = configuration_t::get().load_defaults();
    std::cout << cfg << std::endl;
    executor_t &executor = executor_t::get(cfg);

    executor.enable(true);
    //executor.execute(executor_commands);
    //executor.execute(generate_sin_wave_for_test());
    {
        motion_plan_t mp(cfg);
        mp.gotoxy(distance_t{5.0,0.0,0.0,0.0},2.0)
            .gotoxy(distance_t{5.0,-5.0,0.0,0.0},2.0)
            .gotoxy(distance_t{0.0,-5.0,0.0,0.0},2.0)
            .gotoxy(distance_t{0.0,0.0,0.0,0.0},2.0);
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
        //auto acc = mp.get_accelerations();
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

