#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "mbed.h"
#include "rtos.h"

#define CTRL_PERIOD 	0.02 	//50Hz
#define CTRL_PERIOD_MS 	20		//50Hz

namespace controller {

    extern float duty_cycle;
    extern void nextStep();
}

#endif