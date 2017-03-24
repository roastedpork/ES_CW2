#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "mbed.h"
#include "rtos.h"

//50Hz update cycle for the controller
#define CTRL_PERIOD 	0.02 	
#define CTRL_PERIOD_MS 	20	

namespace controller {
	// Shared resources
	extern rtos::Mutex controller_mutex;
    extern float duty_cycle;

    // Threading function
    extern void nextStep();
}

#endif