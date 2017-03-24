#ifndef ODOMETER_H
#define ODOMETER_H

#include "mbed.h"
#include "rtos.h"

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8  

// Calculations will be in terms of rev and rev/s
#define TICK_RES 8.547e-03
#define HEX_RES 0.166666667

// Odometer update period
#define ODMT_PERIOD_MS 10

namespace odometer {
	// used by others for readRotorState()
	extern InterruptIn I1;
	extern InterruptIn I2;
	extern InterruptIn I3;

	// Debug variables 
	extern int debug_a;
	extern int debug_b;

	// Shared reseources written by odometer
	extern float position;
	extern float velocity;
	extern rtos::Mutex odometer_mutex;

	// Initialization and threading function
	extern void init();
	extern void updateState();
}



#endif