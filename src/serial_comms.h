#ifndef SERIAL_COMMS_H
#define SERIAL_COMMS_H

#include "mbed.h"
#include "rtos.h"


namespace serial_comms {

extern Serial pc;
extern float target_pos;
extern float target_vel;
extern bool update;

extern void init();
extern void getTargets();
}

#endif