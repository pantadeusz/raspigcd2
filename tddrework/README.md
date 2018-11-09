# raspigcd2

[![Build Status](https://travis-ci.org/pantadeusz/raspigcd2.svg?branch=master)](https://travis-ci.org/pantadeusz/raspigcd2)

The second attempt to raspberry pi gcode interpreter


## Short description

This repository contains my experiments on raspberry pi gcode interpreter

It will treat gcode as a complete object and execute it accordingly

the thread will be initialized on gcode exec

# Style

try to maintain stl style with custom types named wit _t as the postfix

tests are in separate files

# Units

Most of units are in SI standard. The velocity and distance, when not marked differently, is presented in mm/s and mm.

# General interpretation of Gcodes

M codes marks separate sections of gcode
G codes are processed as a whole

This means that this kind of gcode program:

```gcode
M3
M17
G1Z-1
G1X10
M5
G0Z10
G0X0
G0Z0
M18
```

will  execute as following parts interpreted as separate executions of motor thread:

 * M3
 * M17
 * G1Z-1, G1X10
 * M5
 * G0Z10, G0X0, G0Z0
 * M18

## Compilation options

You can define ```STEPPING_DELAY_SLEEP_UNTIL``` to disable busy loop in timers. This definition will reduce cpu utilization, but increase jitter.
