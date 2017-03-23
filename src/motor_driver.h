#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "mbed.h"
#include "rtos.h"

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4			//0x01
#define L1Hpin D5			//0x02
#define L2Lpin D3			//0x04
#define L2Hpin D6			//0x08
#define L3Lpin D9			//0x10
#define L3Hpin D10			//0x20


#define STALL_WAIT		3000 	// wait for 3s whenever changing to a POS/VEL/PV opcode
#define PWM_PERIOD 		0.020 	// 50Hz PWM
#define PWM_PERIOD_MS	20 		// 50Hz PWM
#define PWM_TICK 		0.0002 	// 100 ticks give a full period
#define PWM_TICK_US		200 	// 100 ticks give a full period

namespace driver {

		extern void runMotor();
		extern void init();
}

#endif