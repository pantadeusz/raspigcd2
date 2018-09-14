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

#include <executor_pi_t.hpp>

#include <chrono>

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

// RASPBERRY PI GPIO - docs from http://www.pieter-jan.com/node/15

#include <stdio.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <unistd.h>

// #define BCM2708_PERI_BASE       0x20000000   // raspi 1 //
#define BCM2708_PERI_BASE 0x3F000000 // raspi 3 //
#define GPIO_BASE (BCM2708_PERI_BASE + 0x200000)

#define BLOCK_SIZE (4 * 1024)

// IO Acces
struct bcm2835_peripheral
{
    unsigned long addr_p;
    int mem_fd;
    void *map;
    volatile unsigned int *addr;
};

struct bcm2835_peripheral gpio = {GPIO_BASE, 0, 0, 0};

// Exposes the physical address defined in the passed structure using mmap on /dev/mem
int map_peripheral(struct bcm2835_peripheral *p)
{
    // Open /dev/mem
    if ((p->mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0)
    {
        throw std::runtime_error("Failed to open /dev/mem, try checking permissions.\n");
    }

    p->map = mmap(
        NULL,
        BLOCK_SIZE,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        p->mem_fd, // File descriptor to physical memory virtual file '/dev/mem'
        p->addr_p  // Address in physical map that we want this memory block to expose
    );

    if (p->map == MAP_FAILED)
    {
        throw std::runtime_error("map_peripheral failed");
    }

    p->addr = (volatile unsigned int *)p->map;

    return 0;
}

void unmap_peripheral(struct bcm2835_peripheral *p)
{
    munmap(p->map, BLOCK_SIZE);
    close(p->mem_fd);
}

#define INP_GPIO(g) *(gpio.addr + ((g) / 10)) &= ~(7 << (((g) % 10) * 3))
#define OUT_GPIO(g) *(gpio.addr + ((g) / 10)) |= (1 << (((g) % 10) * 3))
#define SET_GPIO_ALT(g, a) *(gpio.addr + (((g) / 10))) |= (((a) <= 3 ? (a) + 4 : (a) == 4 ? 3 : 2) << (((g) % 10) * 3))

#define GPIO_SET *(gpio.addr + 7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio.addr + 10) // clears bits which are 1 ignores bits which are 0

#define GPIO_READ(g) *(gpio.addr + 13) &= (1 << (g))

#define GET_GPIO(g) (*(gpio.addr + 13) & (1 << g)) // 0 if LOW, (1<<g) if HIGH

#define GPIO_PULL *(gpio.addr + 37)     // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio.addr + 38) // Pull up/pull down clock

namespace raspigcd
{

executor_pi_t::executor_pi_t()
{
    configuration_t &configuration = configuration_t::get();
    if (map_peripheral(&gpio) == -1) {
            throw std::invalid_argument("Failed to map the physical GPIO registers into the virtual memory space.");
    }
    // Define pin 7 as output
    for (auto c : configuration.hardware.steppers)
    {
        INP_GPIO(c.step);
        OUT_GPIO(c.step);
        INP_GPIO(c.dir);
        OUT_GPIO(c.dir);
        INP_GPIO(c.en);
        OUT_GPIO(c.en);
    }
    enable(false);
}

executor_pi_t::~executor_pi_t()
{
    unmap_peripheral(&gpio);
}

executor_pi_t &executor_pi_t::get() {
    static executor_pi_t instance;
    return instance;
}



/** from Gordon's Wiring Pi */
/* void dmseconsHard (unsigned int howLong)
{
  *(timer + TIMER_LOAD)    = howLong ;
  *(timer + TIMER_IRQ_CLR) = 0 ;

  while (*timerIrqRaw == 0)
    ;
}   

void dmsecons (unsigned int howLong)
{
  struct timespec sleeper, dummy ;
  
  if (howLong ==   0)
    return ;
  else if (howLong  < 100)
    delayMicrosecondsHard (howLong) ;
  else
  {
    sleeper.tv_sec  = 0 ;
    sleeper.tv_nsec = (long)(howLong * 1000) ;
    nanosleep (&sleeper, &dummy) ;
  } 
}
/ * end of copied code */

int executor_pi_t::execute(const std::vector<executor_command_t> &commands)
{
    configuration_t conf = configuration_t::get();
    auto steppers = conf.hardware.steppers;
    // commands are in fomat step, dir
    unsigned int step_clear =
            (1 << steppers[0].step) |
            (1 << steppers[1].step) |
            (1 << steppers[2].step) |
            (1 << steppers[3].step);

    auto ttime = std::chrono::microseconds((unsigned long) conf.tick_duration*1000000);
    auto t = std::chrono::system_clock::now();
    auto nextT = ttime + t;
    for (auto c : commands)
    {
        // step direction
        unsigned int dir_set =
            (c.b.dir0 << steppers[0].dir) |
            (c.b.dir1 << steppers[1].dir) |
            (c.b.dir2 << steppers[2].dir) |
            (c.b.dir3 << steppers[3].dir);
        unsigned int dir_clear =
            ((1 - c.b.dir0) << steppers[0].dir) |
            ((1 - c.b.dir1) << steppers[1].dir) |
            ((1 - c.b.dir2) << steppers[2].dir) |
            ((1 - c.b.dir3) << steppers[3].dir);

        // shoud do step?
        unsigned int step_set =
            (c.b.step0 << steppers[0].step) |
            (c.b.step1 << steppers[1].step) |
            (c.b.step2 << steppers[2].step) |
            (c.b.step3 << steppers[3].step);

        // first set directions
        GPIO_SET = dir_set;
        GPIO_CLR = dir_clear;
        {volatile int delayloop = 1000; while (delayloop--);}
        // set step to do
        GPIO_SET = step_set;
        {volatile int delayloop = 1000; while (delayloop--);}
        // clear all steps
        GPIO_CLR = step_clear;
        {volatile int delayloop = 1000; while (delayloop--);}
        std::this_thread::sleep_until(nextT);
        t = nextT;
        ttime + t;
    }
    return 0;
}

void executor_pi_t::enable(bool en)
{
    configuration_t &configuration = configuration_t::get();
    for (auto c : configuration.hardware.steppers)
    {
        if (en)
        {
            GPIO_CLR = 1 << c.en;
        }
        else
        {
            GPIO_SET = 1 << c.en;
        }
    }
}

void executor_pi_t::set_position(const std::array<int,4> &steps) {
    steps_from_origin_ = steps;
}
std::array<int,4> executor_pi_t::get_position() const {
    return steps_from_origin_;
}


} // namespace raspigcd
