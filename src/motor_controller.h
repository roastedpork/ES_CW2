#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "mbed.h"
#include "rtos.h"

#define CTRL_PERIOD 0.05

namespace controller {

    extern float duty_cycle;
    extern void nextStep();
}

#endif