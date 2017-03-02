#ifndef SERIAL_COMMS_H
#define SERIAL_COMMS_H

#include "mbed.h"
#include "rtos.h"


namespace serial_comms {

enum update_t {
	OP_NIL = 0,
	OP_POS = 1,
	OP_VEL = 2,
	OP_PV  = 3
};


extern Serial pc;
extern float target_pos;
extern float target_vel;
extern bool update;

extern void init();
extern void getTargets();
}

#endif