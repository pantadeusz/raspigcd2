#include <configuration.hpp>
#include <configuration_json.hpp>
#include <hardware_stepping.hpp>
#include <movement_variable_speed.hpp>

#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>

#include <chrono>
#include <thread>
#include <vector>

using namespace raspigcd;
using namespace raspigcd::configuration;
using namespace raspigcd::hardware;
using namespace raspigcd::movement;

class low_steppers_fake : public hardware::low_steppers
{
public:
    int counters[RASPIGCD_HARDWARE_DOF];
    std::vector<bool> enabled;
    void do_step(const single_step_command* b)
    {
        for (int i = 0; i < RASPIGCD_HARDWARE_DOF; i++) {
            if (b[i].step == 1) {
                counters[i] += b[i].dir * 2 - 1;
            }
        }
    };
    void enable_steppers(const std::vector<bool> en)
    {
        enabled = en;
    };
    low_steppers_fake()
    {
        for (int i = 0; i < RASPIGCD_HARDWARE_DOF; i++) {
            counters[i] = 0;
        }
        enabled = std::vector<bool>(false, RASPIGCD_HARDWARE_DOF);
    }
};


TEST_CASE("Movement variable speed", "[movement][variable_speed]")
{
    std::shared_ptr<low_steppers> lsfake(new low_steppers_fake());
    configuration::global cfg;
    cfg.load_defaults();
    cfg.tick_duration_us = 60;
    cfg.max_no_accel_velocity_mm_s = {5, 5, 5, 5};
    double max_speed_no_accel = cfg.max_no_accel_velocity_mm_s[0];
    double acceleration = 100;
    std::shared_ptr<motor_layout> motor_layout_ = motor_layout::get_instance(cfg);
    movement::variable_speed variable_speed_driver(motor_layout_, max_speed_no_accel, acceleration, 150, cfg.tick_duration());
    stepping_simple_timer stepping(cfg, lsfake);

    distance_t start_coord = {0, 0, 0, 0};
    steps_t steps = motor_layout_.get()->cartesian_to_steps(start_coord);
    distance_t goal_coord = {-1, 1, 2, 0};
    double velocity = 30; // mm/s


    SECTION("Run empty program")
    {
        int n = 0;
        stepping.exec({0, 0, 0, 0}, {}, [&](const auto&) { n++; });
        REQUIRE(n == 0);
    }

    SECTION("Intentions to set of acceleration sequences for empty")
    {
        std::list<var_speed_intentions_t> intended_moves;
        auto move_segments = variable_speed_driver.movement_intet_to_point_speeds(intended_moves);
        REQUIRE(move_segments.size() == 0);
    }
    SECTION("Intention with one value that does not need to accelerate")
    {
        std::list<var_speed_intentions_t> intended_moves;
        intended_moves.push_back({.p0 = {0, 0, 0, 0},
            .p1 = {2, 3, 4, 0},
            .intended_speed = 3});
        auto move_segments = variable_speed_driver.movement_intet_to_point_speeds(intended_moves);
        REQUIRE(move_segments.size() == 2);
        REQUIRE(move_segments.front().v0 == move_segments.back().v0);
        REQUIRE(move_segments.front().p == intended_moves.back().p0);
        REQUIRE(move_segments.back().p == intended_moves.back().p1);
    }
    SECTION("Intention with multiple values that does not need to accelerate")
    {
        std::list<var_speed_intentions_t> intended_moves;
        intended_moves.push_back({.p0 = {0, 0, 0, 0},
            .p1 = {2, 3, 4, 0},
            .intended_speed = 3});
        intended_moves.push_back({.p0 = {2, 3, 4, 0},
            .p1 = {1, 2, 3, 0},
            .intended_speed = 2});
        intended_moves.push_back({.p0 = {1, 2, 3, 0},
            .p1 = {2, 3, 4, 0},
            .intended_speed = 1});
        auto move_segments = variable_speed_driver.movement_intet_to_point_speeds(intended_moves);

        std::vector<var_speed_intentions_t> im_vect(intended_moves.begin(), intended_moves.end());
        std::vector<var_speed_pointspeed_t> result_vect(move_segments.begin(), move_segments.end());
        REQUIRE(result_vect.size() == 6);
        REQUIRE(result_vect[0].v0 == result_vect[1].v0);
        REQUIRE(result_vect[1].v0 != result_vect[2].v0);
        REQUIRE(result_vect[2].v0 == result_vect[3].v0);
        REQUIRE(result_vect[3].v0 != result_vect[4].v0);
        REQUIRE(result_vect[4].v0 == result_vect[5].v0);

        REQUIRE(result_vect[0].p == im_vect[0].p0);
        REQUIRE(result_vect[1].p == im_vect[0].p1);
        REQUIRE(result_vect[2].p == im_vect[1].p0);
        REQUIRE(result_vect[3].p == im_vect[1].p1);
        REQUIRE(result_vect[4].p == im_vect[2].p0);
        REQUIRE(result_vect[5].p == im_vect[2].p1);

        REQUIRE(result_vect[0].max_v == im_vect[0].intended_speed);
        REQUIRE(result_vect[1].max_v == im_vect[0].intended_speed);
        REQUIRE(result_vect[2].max_v == im_vect[1].intended_speed);
        REQUIRE(result_vect[3].max_v == im_vect[1].intended_speed);
        REQUIRE(result_vect[4].max_v == im_vect[2].intended_speed);
        REQUIRE(result_vect[5].max_v == im_vect[2].intended_speed);
    }
    SECTION("Intention with one value that does need to accelerate and break and do some constant segment")
    {
        std::list<var_speed_intentions_t> intended_moves;
        intended_moves.push_back({.p0 = {0, 0, 0, 0},
            .p1 = {22, 32, 42, 0},
            .intended_speed = 20});
        auto move_segments = variable_speed_driver.movement_intet_to_point_speeds(intended_moves);

        REQUIRE(move_segments.size() == 4);
        std::vector<var_speed_intentions_t> im_vect(intended_moves.begin(), intended_moves.end());
        std::vector<var_speed_pointspeed_t> result_vect(move_segments.begin(), move_segments.end());

        REQUIRE(result_vect[0].p == im_vect[0].p0);
        REQUIRE(result_vect[3].p == im_vect[0].p1);

        REQUIRE(result_vect[0].v0 == max_speed_no_accel);
        REQUIRE(result_vect[1].v0 > max_speed_no_accel);
        REQUIRE(result_vect[2].v0 > max_speed_no_accel);
        REQUIRE(result_vect[3].v0 == max_speed_no_accel);

        REQUIRE(result_vect[0].max_v == im_vect[0].intended_speed);
        REQUIRE(result_vect[1].max_v == im_vect[0].intended_speed);
        REQUIRE(result_vect[2].max_v == im_vect[0].intended_speed);
        REQUIRE(result_vect[3].max_v == im_vect[0].intended_speed);

        double full_length = std::sqrt((im_vect[0].p1 - im_vect[0].p0).length2());
        double full_length_of_sum =
            std::sqrt((result_vect[1].p - result_vect[0].p).length2()) +
            std::sqrt((result_vect[2].p - result_vect[1].p).length2()) +
            std::sqrt((result_vect[3].p - result_vect[2].p).length2());
        REQUIRE(full_length == Approx(full_length_of_sum));
    }

    SECTION("Intention with two values that does need to accelerate and break and do some constant segment")
    {
        std::list<var_speed_intentions_t> intended_moves;
        intended_moves.push_back({.p0 = {0, 0, 0, 0},
            .p1 = {22, 32, 42, 0},
            .intended_speed = 20});
        intended_moves.push_back({.p0 = {22, 32, 42, 0},
            .p1 = {0, 0, 0, 0},
            .intended_speed = 20});
        auto move_segments = variable_speed_driver.movement_intet_to_point_speeds(intended_moves);

        REQUIRE(move_segments.size() == 7);
        std::vector<var_speed_intentions_t> im_vect(intended_moves.begin(), intended_moves.end());
        std::vector<var_speed_pointspeed_t> result_vect(move_segments.begin(), move_segments.end());

        REQUIRE(result_vect[0].p == im_vect[0].p0);
        REQUIRE(result_vect[3].p == im_vect[0].p1);

        REQUIRE(result_vect[0].v0 == max_speed_no_accel);
        REQUIRE(result_vect[1].v0 > max_speed_no_accel);
        REQUIRE(result_vect[2].v0 > max_speed_no_accel);
        REQUIRE(result_vect[3].v0 == max_speed_no_accel);

        double full_length = std::sqrt((im_vect[0].p1 - im_vect[0].p0).length2()) + std::sqrt((im_vect[1].p1 - im_vect[1].p0).length2());
        double full_length_of_sum =
            std::sqrt((result_vect[1].p - result_vect[0].p).length2()) +
            std::sqrt((result_vect[2].p - result_vect[1].p).length2()) +
            std::sqrt((result_vect[3].p - result_vect[2].p).length2()) +

            std::sqrt((result_vect[4].p - result_vect[3].p).length2()) +
            std::sqrt((result_vect[5].p - result_vect[4].p).length2()) +
            std::sqrt((result_vect[6].p - result_vect[5].p).length2());
        auto ep = result_vect[0];
        for (auto e : result_vect) {
            double T;
            if (ep.accel == 0) {
                T=std::sqrt((e.p - ep.p).length2())/ep.v0;
            } else {
                T = (e.v0 - ep.v0) / ep.accel;
            }
            double sReal = std::sqrt((e.p - ep.p).length2());
            double sShouldBe = ep.v0 * T + ep.accel * T * T / 2;
            REQUIRE(sReal == Approx(sShouldBe));
            ep = e;
        }
        REQUIRE(full_length == Approx(full_length_of_sum));
    }

    SECTION("Two segments with max speed over max speed no accel")
    {
        std::list<var_speed_intentions_t> intended_moves;
        intended_moves.push_back({.p0 = {0, 0, 0, 0},
            .p1 = {22, 32, 42, 0},
            .intended_speed = 20});
        intended_moves.push_back({.p0 = {22, 32, 42, 0},
            .p1 = {0, 0, 0, 0},
            .intended_speed = 20});
        auto move_segments = variable_speed_driver.movement_intet_to_point_speeds(intended_moves);
        for (auto &e : move_segments) {
            REQUIRE(e.max_v == Approx(20.0));
        }
    }
    SECTION("Two segments with max speed over max speed 20 and 15")
    {
        std::list<var_speed_intentions_t> intended_moves;
        intended_moves.push_back({.p0 = {0, 0, 0, 0},
            .p1 = {22, 32, 42, 0},
            .intended_speed = 15});
        intended_moves.push_back({.p0 = {22, 32, 42, 0},
            .p1 = {0, 0, 0, 0},
            .intended_speed = 20});
        auto move_segments = variable_speed_driver.movement_intet_to_point_speeds(intended_moves);
        int i = 0;
        for (auto &e : move_segments) {
            //std::cout << "E: " << e.max_v << " " << e.v0 << " " << e.accel << " " << e.p << std::endl;
            if (i < 3) {
                REQUIRE(e.max_v == Approx(15.0));
            } else {
                REQUIRE(e.max_v == Approx(20.0));
            }
            i++;
        }
    }

    SECTION("Two segments with max speed over max speed 2 and 15")
    {
        std::list<var_speed_intentions_t> intended_moves;
        intended_moves.push_back({.p0 = {0, 0, 0, 0},
            .p1 = {22, 32, 42, 0},
            .intended_speed = 2});
        intended_moves.push_back({.p0 = {22, 32, 42, 0},
            .p1 = {0, 0, 0, 0},
            .intended_speed = 15});
        auto move_segments = variable_speed_driver.movement_intet_to_point_speeds(intended_moves);
        int i = 0;
        for (auto &e : move_segments) {
            std::cout << "E: " << e.max_v << " " << e.v0 << " " << e.accel << " " << e.p << std::endl;
            if (i < 1) {
                REQUIRE(e.max_v == Approx(2.0));
            } else {
                REQUIRE(e.max_v == Approx(15.0));
            }
            i++;
        }
    }


    SECTION("Acceleration and break should give correct coordinates and velocities")
    {
        std::list<var_speed_intentions_t> intended_moves;
        intended_moves.push_back({.p0 = {0, 0, 0, 0},
            .p1 = {22, 32, 42, 0},
            .intended_speed = 2});
        intended_moves.push_back({.p0 = {22, 32, 42, 0},
            .p1 = {0, 0, 0, 0},
            .intended_speed = 15});
        auto move_segments = variable_speed_driver.movement_intet_to_point_speeds(intended_moves);
        distance_t current_position = {0, 0, 0, 0};

        var_speed_pointspeed_t &pe = move_segments.front();
        for (auto &e : move_segments) {
            //double sShouldBe = ep.v0 * T + ep.accel * T * T / 2;
            if (&pe != &e) {
                auto vect = e.p - pe.p;
                double l = std::sqrt(vect.length2());
                vect = vect/l;
                double dt = cfg.tick_duration();
                double v = pe.v0;

                // while ((current_position-pe.p).length2() < l*l) {
                //     current_position = current_position + vect * v * dt + vect * v * pe.accel * 0.5 * dt * dt;
            	// 	v = v + pe.accel * dt;
                // }
                for (double T = 0; ((current_position-pe.p).length2() < l*l); T+= dt) {
                    current_position = pe.p + vect*(pe.v0 * T + pe.accel * T * T / 2); // simulate movement
                    v = pe.v0 + pe.accel*T;
                } 
                for (int i = 0; i < 4; i++) {
                    REQUIRE(current_position[i] == Approx(e.p[i]).margin(0.001));
                }
                
                REQUIRE(v == Approx(e.v0).margin(0.75));
                //std::cout << "PP : " << current_position << " v: " << v <<  " " << pe.accel <<  " ?? " << e.p << std::endl;
            }
            pe = e;
        }
    }

}
