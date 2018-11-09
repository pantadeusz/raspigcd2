#include <motion_plan_t.hpp>

namespace raspigcd {
int motion_plan_t::steps_to_do(const steps_t& steps_, const steps_t& destination_steps_)
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
std::vector<executor_command_t> motion_plan_t::chase_steps(const steps_t& steps_, steps_t destination_steps_)
{
    //auto &cfg = configuration_t::get();
    std::vector<executor_command_t> ret;
    ret.reserve(steps_to_do(steps_, destination_steps_)*2+32);
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

const std::vector<executor_command_t> motion_plan_t::get_motion_plan() const
{
    std::vector<executor_command_t> _motion_plan;
    if (_motion_fragments.size() == 0) return _motion_plan;
    // calculate angle between two vectors
    auto vectors_angle = [](auto u, auto v) {
        auto dotprod = (u * v).sumv();
        if (dotprod == 0) return 3.14159265358979323846 / 2.0;
        return acos(dotprod / (std::sqrt(u.length2()) * std::sqrt(v.length2())));
    };

    // generates steps with dt as a time frame, with initial velocity v, and that accelerates with a mm/s2 along given vector.
    // returns vector of commands, and finishing velocity
    auto move_to_position_with_given_accel = [&, this](double dt, double v, double a, double length, distance_t norm_vect) -> std::pair<std::vector<executor_command_t>, double> {
        std::vector<executor_command_t> ret2;
        std::list<executor_command_t> ret;
        //ret.reserve(1000000);

        steps_t steps(0, 0, 0, 0);
        distance_t position(0, 0, 0, 0);
        for (double s = 0; s <= length; s += v * dt) {
            position = position + (norm_vect * v * dt);
            auto psteps = motor_layout->cartesian_to_steps(position);
            auto st = chase_steps(steps, psteps);
            steps = psteps;
            ret.insert(ret.end(), st.begin(), st.end());
            if ((v + a * dt) <= 0) {
                // std::cerr << "err: v=" << v << " <= 0; " << s << " " << length << std::endl;
                break;
            } else {
                v = v + a * dt;
            }
        }
        ret2.reserve(ret.size());
        for (auto &e : ret) {
            if (ret2.size()>0) {
                if (ret2.back().b == e.b) {
                    ret2.back().cmnd.repeat++;
                } else {
                    ret2.push_back(e);
                }
            } else {
                ret2.push_back(e);
            }
        }
        ret2.shrink_to_fit();
        return {ret2, v};
    };


    // given speed, target speed and acceleration, it calculate distance that it will be accelerating
    auto accleration_length_calc = [](auto speed_0, auto speed_1, auto acceleration) {
        double t_AM = (speed_1 - speed_0) / acceleration;
        double l_AM = std::abs(speed_0 * t_AM + acceleration * t_AM * t_AM / 2.0);
        return l_AM;
    };

    // calulates maximal linear acceleration given the maximal accelerations for each axis, and the normal vecotr of intended move
    auto calculate_maximal_acceleration = [](auto max_accelerations_mm_s2, auto norm_vect) {
        double average_max_accel = 0;
        {
            double average_max_accel_sum = 0;
            for (int i = 0; i < DEGREES_OF_FREEDOM; i++) {
                average_max_accel += max_accelerations_mm_s2[i] * norm_vect[i];
                average_max_accel_sum += norm_vect[i];
            }
            average_max_accel = average_max_accel / average_max_accel_sum;
        }
        return average_max_accel;
    };


    // calculates possible maximal speed given work speed, acceleration, breaking, and length of the work fragment
    // returns tuple std::make_tuple(l_AM, l_M, l_MB) that represents accelerating part length, constant speed lenght and breaking lenght.
    // note that it can give l_M negative, that means it could not find any valid lengths for given parameters
    auto calculate_velocity_target = [accleration_length_calc](double length, double prev_speed, double next_speed, double max_velocity, double average_max_accel) {
        // binary search for best max velocity that fits
        double l_AM = 0;
        double l_M = -1;
        double l_MB = 0;

        double current_velocity_target = max_velocity;
        for (double current_velocity_target_range = max_velocity / 2.0; 
            current_velocity_target_range > 0.001; 
            current_velocity_target_range /= 2.0) {
            auto lAM = (prev_speed < current_velocity_target) ? accleration_length_calc(prev_speed, current_velocity_target, average_max_accel) : 0.0;
            auto lMB = (current_velocity_target > next_speed) ? accleration_length_calc(current_velocity_target, next_speed, average_max_accel) : 0.0;
            auto lM = length - (lMB + lAM);

            if (lM < 0.0) {
                current_velocity_target -= current_velocity_target_range;
            } else if (lM > 0.0) {
                l_AM = lAM;
                l_MB = lMB;
                l_M = lM;
                current_velocity_target += current_velocity_target_range;
                if (current_velocity_target > max_velocity) {
                    current_velocity_target = max_velocity;
                    current_velocity_target_range = 0.0;
                }
            } else {
                l_AM = lAM;
                l_MB = lMB;
                l_M = lM;
                break;
            }
        }
        return std::make_tuple(l_AM, l_M, l_MB);
    };


    auto mfrag_to_normalvect_and_length = [](distance_t source, distance_t destination) {
        auto diff_vect = destination - source; // difference
        double length2 = diff_vect.length2();  // length of vector in mm squared
        double length = std::sqrt(length2);    // length of vector in mm
        auto norm_vect = diff_vect / length;   // direction with length 1mm
        return std::make_tuple(norm_vect, length);
    };

    auto gotoxy = [&, this](double prev_speed, const motion_fragment_t& mfrag, double next_speed) {
        double length_remaining; // length of vector in mm
        distance_t norm_vect;    // direction with length 1mm
        std::tie(norm_vect, length_remaining) = mfrag_to_normalvect_and_length(mfrag.source, mfrag.destination);
        double dt = _cfg->tick_duration; // time step
        std::vector<executor_command_t> local_motion_plan;
        local_motion_plan.reserve(100000);


        // calculate maximal acceleration
        double average_max_accel = calculate_maximal_acceleration(_cfg->layout.max_accelerations_mm_s2, norm_vect);

        // binary search for best max velocity that fits
        double l_AM, l_M, l_MB;
        std::tie(l_AM, l_M, l_MB) = calculate_velocity_target(length_remaining, prev_speed, next_speed, mfrag.max_velocity, average_max_accel);

        if (l_M < 0.0) throw std::invalid_argument("impossible accelerations" + std::to_string(l_M));
        double temporary_velocity = prev_speed;
        auto end_position_actual = motor_layout->cartesian_to_steps(mfrag.source);
        if (l_AM > 0) { /// acceleration sequence
//            std::cerr << "move_to_position_with_given_accel(" << dt << ", " << prev_speed << ", " << average_max_accel << ", " << l_AM << ", norm_vect)" << std::endl;
            auto st_accel = move_to_position_with_given_accel(dt, prev_speed, average_max_accel, l_AM, norm_vect);
            local_motion_plan.insert(local_motion_plan.end(), st_accel.first.begin(), st_accel.first.end());
            temporary_velocity = st_accel.second;
            end_position_actual = end_position_actual + step_sequence_executor_t::commands_to_steps(st_accel.first);
            std::tie(norm_vect, length_remaining) = mfrag_to_normalvect_and_length(motor_layout->steps_to_cartesian(end_position_actual), mfrag.destination);
        }
        if (l_M > 0) { /// constant speed sequence
//            std::cerr << "move_to_position_with_given_accel(" << dt << ", " << temporary_velocity << ", " << 0 << ", " << l_M << ", norm_vect)" << std::endl;
            auto st_const = move_to_position_with_given_accel(dt, temporary_velocity, 0, l_M, norm_vect);
            local_motion_plan.insert(local_motion_plan.end(), st_const.first.begin(), st_const.first.end());
            temporary_velocity = st_const.second;
            end_position_actual = end_position_actual + step_sequence_executor_t::commands_to_steps(st_const.first);
            std::tie(norm_vect, length_remaining) = mfrag_to_normalvect_and_length(motor_layout->steps_to_cartesian(end_position_actual), mfrag.destination);
        }
        if (l_MB > 0) { /// break sequence
//            std::cerr << "move_to_position_with_given_accel(" << dt << ", " << temporary_velocity << ", " << (-average_max_accel) << ", " << l_MB << ", " << norm_vect << ")" << std::endl;
            auto st_decel = move_to_position_with_given_accel(dt, temporary_velocity, -average_max_accel, l_MB, norm_vect);
            local_motion_plan.insert(local_motion_plan.end(), st_decel.first.begin(), st_decel.first.end());
            temporary_velocity = st_decel.second;
            end_position_actual = end_position_actual + step_sequence_executor_t::commands_to_steps(st_decel.first);
            std::tie(norm_vect, length_remaining) = mfrag_to_normalvect_and_length(motor_layout->steps_to_cartesian(end_position_actual), mfrag.destination);
        }
        /// fix steps count (probably 1 step in some direction to correct errors)
        auto distance_accuracy = std::sqrt(motor_layout->steps_to_cartesian({1, 1, 1, 1}).length2());
//        std::cerr << "2*distance_accuracy " << (2 * distance_accuracy) << " " << length_remaining << std::endl;
        if (length_remaining * length_remaining > 1.5 * distance_accuracy) {
//            std::cerr << "FIX " << length_remaining << " move_to_position_with_given_accel(" << dt << ", " << temporary_velocity << ", " << 0 << ", " << length_remaining << ", " << norm_vect << ")" << std::endl;
            auto st_fix = move_to_position_with_given_accel(dt, temporary_velocity, 0, length_remaining, norm_vect);
            local_motion_plan.insert(local_motion_plan.end(), st_fix.first.begin(), st_fix.first.end());
            temporary_velocity = st_fix.second;
        }
        local_motion_plan.shrink_to_fit();
        _motion_plan.insert(_motion_plan.end(), local_motion_plan.begin(), local_motion_plan.end());
        return step_sequence_executor_t::commands_to_steps(local_motion_plan);
    };


    steps_t steps = motor_layout->cartesian_to_steps(_motion_fragments.front().source);
    // std::cerr << "steps: " << steps << std::endl;
    std::vector<motion_fragment_t> motion_fragments_vector(_motion_fragments.begin(), _motion_fragments.end());

    // make breakings as small as possible
    for (unsigned int i = 1; i < motion_fragments_vector.size() - 1; i++) {
        auto& m = motion_fragments_vector[i];
        // create previous and next motion fragments. If this is first or last element, then make fragment that is in opposite direction to current element
        auto prev_fragment = motion_fragments_vector[i - 1];
        auto next_fragment = motion_fragments_vector[i + 1];
        double coeffA = vectors_angle(prev_fragment.destination - prev_fragment.source, m.destination - m.source) / 3.14159265358979323846;
        double coeffB = vectors_angle(m.destination - m.source, next_fragment.destination - next_fragment.source) / 3.14159265358979323846;
        std::cerr << "coeffA " << coeffA << "  coeffB " << coeffB << std::endl;

        // min speed no accel
        double prev_speed_dst = std::min(prev_fragment.max_velocity, m.max_velocity); // fist - the speed cannot exceed maximal speed from previous step
        if (coeffA > 0.5)
            prev_speed_dst = _cfg->layout.max_no_accel_velocity_mm_s;
        else
            prev_speed_dst = (coeffA * _cfg->layout.max_no_accel_velocity_mm_s + (0.5 - coeffA) * prev_speed_dst) / 0.5;

        std::cerr << " prev_speed_dst " << prev_speed_dst << "           :: prev_fragment.max_velocity:" << prev_fragment.max_velocity << " m.max_velocity:" << m.max_velocity << std::endl;
        *m.prev_speed.get() = prev_speed_dst;
    }


    // TODO: check if breaks are possible
    // for (unsigned int i = ((int)motion_fragments_vector.size()) - 2; i > 0; i--) {
    // }

    // perform action
    for (unsigned int i = 0; i < motion_fragments_vector.size(); i++) {
        auto mfrag = motion_fragments_vector[i];

        mfrag.source = motor_layout->steps_to_cartesian(steps);
        std::cerr << "mfrag.source: " << mfrag.source << std::endl;
        auto steps_diff = gotoxy(*mfrag.prev_speed.get(), mfrag, *mfrag.next_speed.get()); //.destination, mfrag.max_velocity);
        steps = steps + steps_diff;
    }
    // it must be always accurate
    if (_motion_fragments.size() > 0) {
        auto st = chase_steps(steps, motor_layout->cartesian_to_steps(_motion_fragments.back().destination));
        _motion_plan.insert(_motion_plan.end(), st.begin(), st.end());
    }

    return _motion_plan;
};
const steps_t motion_plan_t::get_steps() const { return motor_layout->cartesian_to_steps(_current_position); };
const distance_t motion_plan_t::get_position() const { return _current_position; };

motion_plan_t& motion_plan_t::set_steps(const steps_t& steps_)
{
    _current_position = motor_layout->steps_to_cartesian(steps_);
    return *this;
};
motion_plan_t& motion_plan_t::set_position(const distance_t& position_)
{
    _current_position = position_;
    return *this;
};

motion_plan_t& motion_plan_t::clear_motion_fragments_buffer()
{
    _motion_fragments.clear();
    return *this;
};

// motion_plan_t &set_motion_plan(const std::vector<executor_command_t>& mp_) {_motion_plan = mp_; return *this;};

motion_plan_t& motion_plan_t::gotoxy(const distance_t& end_position_, const double& velocity_mm_s_)
{
    std::shared_ptr<double> prev_speed;
    std::shared_ptr<double> next_speed(new double);
    *(next_speed.get()) = std::min(velocity_mm_s_, _cfg->layout.max_no_accel_velocity_mm_s);
    if (_motion_fragments.size() == 0) {
        prev_speed = std::shared_ptr<double>(new double);
        *(prev_speed.get()) = std::min(velocity_mm_s_, _cfg->layout.max_no_accel_velocity_mm_s);
    } else {
        prev_speed = _motion_fragments.back().prev_speed;
    }
    motion_fragment_t mf = {
        _current_position,
        end_position_,
        max_velocity : velocity_mm_s_,
        prev_speed : prev_speed,
        next_speed : next_speed
    };
    _motion_fragments.push_back(mf);
    _current_position = end_position_;
    return *this;
}


motion_plan_t::motion_plan_t(configuration_t& configuration)
{
    motor_layout = motor_layout_t::get_and_update_instance(configuration);
    _cfg = &configuration;
}


} // namespace raspigcd
