#ifndef PARSER_H
#define PARSER_H

#include "mbed.h"
#include "rtos.h"

#define BUFF_SIZE 64
#define TUNE_BUFFER 64

namespace parser {

	enum update_t {
		OP_NIL = 0,
		OP_POS = 1,
		OP_VEL = 2,
		OP_PV  = 3,
		OP_TUNE = 4,
		OP_BPM = 5
	};

	// extern rtos::Mutex serial_mutex;
	extern RawSerial pc;
	extern float target_pos;
	extern float target_vel;
	extern float tune_period;
	extern update_t op_code;
	extern int tunes_list[TUNE_BUFFER]; // Buffer of periods, from parsed tune
	extern volatile bool output_ready;

	extern void init();
	extern void pollSerialIn();
}

#endif