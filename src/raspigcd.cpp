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
            executor_command.b[i].dir = ((destination_steps_[i] > steps[i]) ? 1 : 0);// TODO: (not obviosu) * ((cfg.hardware.steppers[i].direction_reverse) ? -1 : 1);
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
    auto &cfg = configuration_t::get();

    std::vector<executor_command_t> executor_commands;
    executor_commands.reserve((::size_t)(T/cfg.tick_duration));
    steps_t steps;

    for (int i = 0; i < 100; i++)
    {
        executor_command_t e;
        e.v = 0;
        executor_commands.push_back(e);
    }

    //    steps[axis] = cfg.hardware.steppers[axis].stepsPerMm*std::cos(0)*amplitude;
    int minimal_step_skip = 1000000000;
    for (double t = 0.0; t < T; t += cfg.tick_duration)
    {
        steps_t new_steps;
        new_steps[axis] = cfg.hardware.steppers[axis].steps_per_mm * std::cos(t * 3.141592653589793238462643 * 5) * amplitude * std::sin(3.141592653589793238462643 * t / T);
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

class motion_plan_t {
protected:
    std::vector<executor_command_t> _motion_plan;
    steps_t _steps;
public:
    const std::vector<executor_command_t> &get_motion_plan() const {return _motion_plan;};
    const steps_t &get_steps() const {return _steps;};
    const distance_t get_position(const distance_t &position_) const {return motor_layout_t::get().steps_to_cartesian(_steps);};

    motion_plan_t &set_steps(const steps_t &steps_) {_steps = steps_;return *this;};
    motion_plan_t &set_position(const distance_t &position_) {_steps = motor_layout_t::get().cartesian_to_steps(position_);return *this;};
    motion_plan_t &set_motion_plan(const std::vector<executor_command_t>& mp_) {_motion_plan = mp_; return *this;};

    motion_plan_t & gotoxy(const steps_t &end_position_, double velocity_mm_s_) {
        static configuration_t &cfg = configuration_t::get();
        auto A = motor_layout_t::get().steps_to_cartesian(_steps);
        auto B = motor_layout_t::get().steps_to_cartesian(end_position_);
        auto diff_vect = B-A;
        double length2 = diff_vect.length2();
        double length = std::sqrt(length2);
        auto norm_vect = diff_vect/length;
        //double ds = 1.0/std::max(std::max(cfg.hardware.steppers[0].steps_per_mm,cfg.hardware.steppers[1].steps_per_mm),cfg.hardware.steppers[2].steps_per_mm);
        double T = length/velocity_mm_s_;
        double dt = cfg.tick_duration;
        double ds = velocity_mm_s_*dt;
        _motion_plan.reserve(_motion_plan.size()+100000);
        for (int i = 0; (i*dt) < T; i++) {
            double t = dt*i;
            double s = ds*i;
            auto p = A + norm_vect*s;
            auto psteps = motor_layout_t::get().cartesian_to_steps(p);
            
            auto st = chase_steps(_steps, psteps);
            _motion_plan.insert(_motion_plan.end(), st.begin(), st.end());
            _steps = psteps;
        }
        {
            auto st = chase_steps(_steps, end_position_);
            _motion_plan.insert(_motion_plan.end(), st.begin(), st.end());
            _steps = end_position_;
        }
        _motion_plan.shrink_to_fit();
        return *this;
    }
    motion_plan_t & gotoxy(const distance_t &end_position_, const double &velocity_mm_s_) {
        return gotoxy(motor_layout_t::get().cartesian_to_steps(end_position_),velocity_mm_s_);
    }

    /**
     * @brief calculates maximal acceleration on plan in mm/s2
     * 
     */
    std::vector < std::array<int, DEGREES_OF_FREEDOM> > get_accelerations(int tticks_count = 500)
    {
        static auto &cfg = configuration_t::get();
/*        auto ticks_per_axis_left = [this](int i, int n, int dof) {
            int s = 0;
            int A = std::max((int)0, (int)(i - (int)(n >> 1)));
            for (int e = i-1; e >= A; e--)
            {
                s += ((int)_motion_plan[e].b[dof].step) * (((int)(_motion_plan[e].b[dof].dir << 1)) - 1);
            }
            return s;
        };
        auto ticks_per_axis_right = [this](int i, int n, int dof) {
            int s = 0;
            int B = std::min((int)_motion_plan.size()-1, (int)(i + (n >> 1)));
            for (int e = i+1; e <= B; e++)
            {
                s += ((int)_motion_plan[e].b[dof].step) * (((int)(_motion_plan[e].b[dof].dir << 1)) - 1);
            }
            return s;
        };
  */      
        
        std::vector < std::array<int, DEGREES_OF_FREEDOM> > velocities(_motion_plan.size());
        int ticks_l[DEGREES_OF_FREEDOM] = {0,0,0,0};
        int ticks_r[DEGREES_OF_FREEDOM] = {0,0,0,0};
        for (int dof = 0; dof < DEGREES_OF_FREEDOM; dof++) {
            for (int i = 1; i <= std::min((int)tticks_count>>1,(int)_motion_plan.size()-1); i++) {
                ticks_r[dof] += ((int)_motion_plan[i].b[dof].step) * (((int)(_motion_plan[i].b[dof].dir << 1)) - 1);
            }
        }
        for (int i = 0; i < _motion_plan.size(); i++)
        {
            for (int dof = 0; dof < DEGREES_OF_FREEDOM; dof++) {
                //if (_motion_plan[i].b[dof].step) velocities[i][dof] = ticks_per_axis_right(i,tticks_count,dof) - ticks_per_axis_left(i,tticks_count,dof);
                if (_motion_plan[i].b[dof].step) velocities[i][dof] = ticks_r[dof] - ticks_l[dof];
                //if ((ticks_r[dof] != ticks_per_axis_right(i,tticks_count,dof)) || (ticks_l[dof] != ticks_per_axis_left(i,tticks_count,dof))) {
                //    std::cerr << "i = " << i << std::endl;
                //    std::cerr << "ticks_r[dof] " << ticks_r[dof] << " != "<<ticks_per_axis_right(i,tticks_count,dof) <<" ticks_per_axis_right(i,tticks_count,dof)" << std::endl;
                //    std::cerr << "ticks_l[dof] " << ticks_l[dof] << " != "<<ticks_per_axis_left(i,tticks_count,dof) <<" ticks_per_axis_left(i,tticks_count,dof)" << std::endl;
                //    throw "bla";
                //}
            }
            for (int dof = 0; dof < DEGREES_OF_FREEDOM; dof++) {
                {
                    auto ticks_r_i = (int)(i+1 + (tticks_count >> 1));
                    if (ticks_r_i < _motion_plan.size()) 
                        ticks_r[dof] += ((int)_motion_plan[ticks_r_i].b[dof].step) * (((int)(_motion_plan[ticks_r_i].b[dof].dir << 1)) - 1);
                    if (i <  _motion_plan.size()) ticks_r[dof] -= ((int)_motion_plan[i+1].b[dof].step) * (((int)(_motion_plan[i+1].b[dof].dir << 1)) - 1);
                }
                {
                    auto ticks_l_i = (int)(i - (tticks_count >> 1));
                    if (ticks_l_i >= 0) 
                        ticks_l[dof] -= ((int)_motion_plan[ticks_l_i].b[dof].step) * (((int)(_motion_plan[ticks_l_i].b[dof].dir << 1)) - 1);
                    //if (i > 0 ) 
                    ticks_l[dof] += ((int)_motion_plan[i].b[dof].step) * (((int)(_motion_plan[i].b[dof].dir << 1)) - 1);
                }
            }
//            std::cout << "dof" << i;
//            for (auto v : velocities[i]) std::cout << " " << v;
//            std::cout  << std::endl;
        }
        return velocities;
    }

    motion_plan_t & fix_accelerations(const int iterations_max = 2, const std::array<int,DEGREES_OF_FREEDOM> &max_diff_to_fix = {4,4,2,4}, const int tticks_count = 250 ) {
        static executor_command_t empty_command = {v:0};
        for (int i = 0; i < iterations_max; i++) {
            std::vector<executor_command_t> new_motion_plan;
            int fixes = 0;
            new_motion_plan.reserve(_motion_plan.size()*2);
            std::vector < std::array<int, DEGREES_OF_FREEDOM> > accels = get_accelerations(tticks_count);
            for (int i = 0; i < accels.size(); i++) {
                for (int dof = 0; dof < DEGREES_OF_FREEDOM; dof++) {
                    if (accels[i][dof] < -max_diff_to_fix[dof]) {
                        new_motion_plan.push_back(empty_command);
                        fixes++;
                    }
                }
                new_motion_plan.push_back(_motion_plan[i]);
                for (int dof = 0; dof < DEGREES_OF_FREEDOM; dof++) {
                    if (accels[i][dof] > max_diff_to_fix[dof]) {
                        
                        new_motion_plan.push_back(empty_command);
                        fixes++;
                    }
                }
            }
            new_motion_plan.shrink_to_fit();
            _motion_plan.clear();
            _motion_plan.reserve(new_motion_plan.size());
            int empty_to_skip = 0;
            for (auto &e : new_motion_plan) {
                if (e.v != 0) break;
                empty_to_skip++;
            }
            _motion_plan.insert(_motion_plan.end(), new_motion_plan.begin() + empty_to_skip, new_motion_plan.end());
            if (fixes == 0) {
                std::cerr << "shorted acceleration fixes " << i << std::endl;
                break;
            }
        }
        return *this;
    }  

    motion_plan_t(const distance_t &accelerations) {

    }
};

int main(int argc, char **argv)
{
    std::vector<std::string> args(argv, argv + argc);

    static auto &cfg = configuration_t::get().load_defaults();
    std::cout << cfg << std::endl;
    executor_t &executor = executor_t::get();

    executor.enable(true);
    //executor.execute(executor_commands);
    //executor.execute(generate_sin_wave_for_test());
    {
        motion_plan_t mp({200,200,100,100});
//        mp.gotoxy(distance_t{5.0,0.0,0.0,0.0},10.0)
//            .gotoxy(distance_t{5.0,-5.0,0.0,0.0},10.0)
//            .gotoxy(distance_t{0.0,-5.0,0.0,0.0},10.0)
//            .gotoxy(distance_t{0.0,0.0,0.0,0.0},10.0);
        mp.gotoxy(distance_t{0.0,0.0,5.0,0.0},10.0)
            .gotoxy(distance_t{0.0,0.0,-5.0,0.0},10.0)
            .gotoxy(distance_t{0.0,0.0,0.0,0.0},10.0);

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

