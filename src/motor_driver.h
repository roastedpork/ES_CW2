#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "mbed.h"
#include "rtos.h"
#include "parser.h"

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4			//0x01
#define L1Hpin D5			//0x02
#define L2Lpin D3			//0x04
#define L2Hpin D6			//0x08
#define L3Lpin D9			//0x10
#define L3Hpin D10			//0x20

// Some defined periods
#define LOW_SPEED_PWM_PERIOD 0.01 // in us
#define SPEED_THRESH 15
#define STALL_WAIT 	5000

// For our own implementation of the PWM
#define PWM_PERIOD 	0.005 	// 200Hz PWM
#define PWM_TICK 	0.00005 // 100 ticks give a full period

namespace driver {

	// Debugging variables, whatever I want it to be
	extern int debug_int;
	extern float debug_f;
	extern parser::update_t debug_op;
	extern parser::update_t debug_curr_op;
	extern parser::update_t debug_new_op;

	// Initialization and threading functions
	extern void runMotor();
	extern void init();
}

#endif