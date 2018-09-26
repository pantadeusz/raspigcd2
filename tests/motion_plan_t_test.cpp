#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>


#include <configuration_t_json.hpp>
#include <motion_plan_t.hpp>
#include <vector>

using namespace raspigcd;

TEST_CASE("Steps calculated by motion_plan_t", "[motion_plan_t]")
{
    SECTION("long compare two methods of path generation")
    {
        std::vector<executor_command_t> planA;
        std::vector<executor_command_t> planB;
        configuration_t cfg;
        cfg.load_defaults();
        {
            motion_plan_t mp(cfg);
            mp.gotoxy(distance_t{5.0, 0.0, 0.0, 0.0}, 20.0)
                .gotoxy(distance_t{5.0, -2.0, 0.0, 0.0}, 20.0)
                .gotoxy(distance_t{5.0, -5.0, 0.0, 0.0}, 5.0)
                .gotoxy(distance_t{0.0, -5.0, 0.0, 0.0}, 20.0)
                .gotoxy(distance_t{0.0, 0.0, 0.0, 0.0}, 20.0);
            planA = mp.get_motion_plan();
        }
        {
            std::vector<executor_command_t> fragment;
            motion_plan_t mp(cfg);
            mp.gotoxy(distance_t{5.0, 0.0, 0.0, 0.0}, 20.0);
            fragment = mp.get_motion_plan();
            planB.insert(planB.end(), fragment.begin(), fragment.end());
            mp.clear_motion_fragments_buffer();

            mp.gotoxy(distance_t{5.0, -2.0, 0.0, 0.0}, 20.0);
            fragment = mp.get_motion_plan();
            planB.insert(planB.end(), fragment.begin(), fragment.end());
            mp.clear_motion_fragments_buffer();

            mp.gotoxy(distance_t{5.0, -5.0, 0.0, 0.0}, 5.0);
            fragment = mp.get_motion_plan();
            planB.insert(planB.end(), fragment.begin(), fragment.end());
            mp.clear_motion_fragments_buffer();

            mp.gotoxy(distance_t{0.0, -5.0, 0.0, 0.0}, 20.0);
            fragment = mp.get_motion_plan();
            planB.insert(planB.end(), fragment.begin(), fragment.end());
            mp.clear_motion_fragments_buffer();

            mp.gotoxy(distance_t{0.0, 0.0, 0.0, 0.0}, 20.0);
            fragment = mp.get_motion_plan();
            planB.insert(planB.end(), fragment.begin(), fragment.end());
        }

        steps_t planAsteps = executor_t::commands_to_steps(planA);
        steps_t planBsteps = executor_t::commands_to_steps(planB);

        REQUIRE(planAsteps == planBsteps);
    }

    SECTION("short compare two methods of path generation")
    {
        std::vector<executor_command_t> planA;
        std::vector<executor_command_t> planB;
        configuration_t cfg;
        cfg.load_defaults();
        {
            motion_plan_t mp(cfg);
            mp.gotoxy(distance_t{5.0, 0.0, 0.0, 0.0}, 20.0)
                .gotoxy(distance_t{5.0, -2.0, 0.0, 0.0}, 20.0)
                .gotoxy(distance_t{5.0, -5.0, 0.0, 0.0}, 5.0);
            planA = mp.get_motion_plan();
        }
        {
            std::vector<executor_command_t> fragment;
            motion_plan_t mp(cfg);
            mp.gotoxy(distance_t{5.0, 0.0, 0.0, 0.0}, 20.0);
            fragment = mp.get_motion_plan();
            planB.insert(planB.end(), fragment.begin(), fragment.end());
            mp.clear_motion_fragments_buffer();

            mp.gotoxy(distance_t{5.0, -2.0, 0.0, 0.0}, 20.0);
            fragment = mp.get_motion_plan();
            planB.insert(planB.end(), fragment.begin(), fragment.end());
            mp.clear_motion_fragments_buffer();

            mp.gotoxy(distance_t{5.0, -5.0, 0.0, 0.0}, 5.0);
            fragment = mp.get_motion_plan();
            planB.insert(planB.end(), fragment.begin(), fragment.end());
            mp.clear_motion_fragments_buffer();
        }

        steps_t planAsteps = executor_t::commands_to_steps(planA);
        steps_t planBsteps = executor_t::commands_to_steps(planB);

        REQUIRE(planAsteps == planBsteps);
    }


    SECTION("alternative method of path generation")
    {
        std::vector<executor_command_t> planA;
        std::vector<executor_command_t> planB;
        configuration_t cfg;
        cfg.load_defaults();
        {
            motion_plan_t mp(cfg);
            mp.gotoxy(distance_t{5.0, 0.0, 0.0, 0.0}, 20.0)
                .gotoxy(distance_t{5.0, -2.0, 0.0, 0.0}, 20.0)
                .gotoxy(distance_t{5.0, -5.0, 0.0, 0.0}, 5.0);
            planA = mp.get_motion_plan();
        }

        {
            distance_t position_;
            {
                std::vector<executor_command_t> fragment;
                motion_plan_t mp(cfg);
                mp.set_position(position_).gotoxy(distance_t{5.0, 0.0, 0.0, 0.0}, 20.0);
                fragment = mp.get_motion_plan();
                planB.insert(planB.end(), fragment.begin(), fragment.end());
                position_ = mp.get_position();
            }
            {
                std::vector<executor_command_t> fragment;
                motion_plan_t mp(cfg);
                mp.set_position(position_).gotoxy(distance_t{5.0, -2.0, 0.0, 0.0}, 20.0);
                fragment = mp.get_motion_plan();
                planB.insert(planB.end(), fragment.begin(), fragment.end());
                position_ = mp.get_position();
            }
            {
                std::vector<executor_command_t> fragment;
                motion_plan_t mp(cfg);
                mp.set_position(position_).gotoxy(distance_t{5.0, -5.0, 0.0, 0.0}, 5.0);
                fragment = mp.get_motion_plan();
                planB.insert(planB.end(), fragment.begin(), fragment.end());
                position_ = mp.get_position();
            }
        }


        steps_t planAsteps = executor_t::commands_to_steps(planA);
        steps_t planBsteps = executor_t::commands_to_steps(planB);

        REQUIRE(planAsteps == planBsteps);
    }


    SECTION("euclidean coordinate system")
    {
        std::vector<executor_command_t> planA;
        std::vector<executor_command_t> planB;
        configuration_t cfg;
        cfg.load_defaults();
        cfg.layout.name = "cartesian";
        {
            motion_plan_t mp(cfg);
            mp.gotoxy(distance_t{5.0, 0.0, 0.0, 0.0}, 20.0)
                .gotoxy(distance_t{5.0, -2.0, 0.0, 0.0}, 20.0)
                .gotoxy(distance_t{5.0, -5.0, 0.0, 0.0}, 5.0);
            planA = mp.get_motion_plan();
        }

        {
            distance_t position_;
            {
                std::vector<executor_command_t> fragment;
                motion_plan_t mp(cfg);
                mp.set_position(position_).gotoxy(distance_t{5.0, 0.0, 0.0, 0.0}, 20.0);
                fragment = mp.get_motion_plan();
                planB.insert(planB.end(), fragment.begin(), fragment.end());
                position_ = mp.get_position();
            }
            {
                std::vector<executor_command_t> fragment;
                motion_plan_t mp(cfg);
                mp.set_position(position_).gotoxy(distance_t{5.0, -2.0, 0.0, 0.0}, 20.0);
                fragment = mp.get_motion_plan();
                planB.insert(planB.end(), fragment.begin(), fragment.end());
                position_ = mp.get_position();
            }
            {
                std::vector<executor_command_t> fragment;
                motion_plan_t mp(cfg);
                mp.set_position(position_).gotoxy(distance_t{5.0, -5.0, 0.0, 0.0}, 5.0);
                fragment = mp.get_motion_plan();
                planB.insert(planB.end(), fragment.begin(), fragment.end());
                position_ = mp.get_position();
            }
        }


        steps_t planAsteps = executor_t::commands_to_steps(planA);
        steps_t planBsteps = executor_t::commands_to_steps(planB);

        REQUIRE(planAsteps == planBsteps);
    }
}
