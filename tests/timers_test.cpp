#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include <vector>
#include <chrono>
#include <thread>


#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <unistd.h>

// using namespace raspigcd;

TEST_CASE( "Experiments for timer delays", "[timers]" ) {


    SECTION( "test for busy delay" ) {

        auto t0 = std::chrono::system_clock::now();
            {
            volatile int delayloop = 50;
            while (delayloop--)
            ;
        }
        {
            volatile int delayloop = 100;
            while (delayloop--)
            ;
        }
        // clear all steps
        {
            volatile int delayloop = 20;
            while (delayloop--)
            ;
        }
        auto t1 = std::chrono::system_clock::now();
        double elaspedTimeMs = std::chrono::duration<double, std::micro>(t1-t0).count();
        std::cerr << "time for tick: " << elaspedTimeMs << " microseconds"<< std::endl;

        REQUIRE(1 == 1);
    }

    SECTION( "test for microsecond delay" ) {
        { // make this thread (if this is a thread) real time
            sched_param sch_params;
            sch_params.sched_priority = sched_get_priority_max(SCHED_RR);

            if (pthread_setschedparam(pthread_self(), SCHED_RR, &sch_params))
            {
            std::cerr << "Warning: Failed to set Thread scheduling : "
                        << std::strerror(errno) << std::endl;
            }
        }
        using namespace std::this_thread; // sleep_for, sleep_until
        using namespace std::chrono; // nanoseconds, system_clock, seconds
        double max_delay_recorded = 0.0;
        for (int i = 0; i < 1000; i++) {
            int n = 0;
            auto t0 = std::chrono::system_clock::now();
            //sleep_for(microseconds(5));
            while (std::chrono::duration<double, std::micro>(std::chrono::system_clock::now()-t0).count() < 5) {n++;};
            auto t1 = std::chrono::system_clock::now();
            double elaspedTimeMs = std::chrono::duration<double, std::micro>(t1-t0).count();
            if (i > 100) if (max_delay_recorded < elaspedTimeMs) {
                max_delay_recorded = elaspedTimeMs;
                std::cerr << "++++ for microseconds(5): " << max_delay_recorded << " microseconds; " << n << std::endl;
            }
        }
        std::cerr << "time for microseconds(5): " << max_delay_recorded << " microseconds"<< std::endl;

        REQUIRE(1 == 1);
    }
}
