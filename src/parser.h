#ifndef PARSER_H
#define PARSER_H

#include "mbed.h"
#include "rtos.h"

// Defined array sizes
#define BUFF_SIZE 64
#define TUNE_BUFFER 16

// Index definitions for the ready array
#define MAIN_INDEX 0
#define ODMT_INDEX 1
#define CTRL_INDEX 2
#define DRVR_INDEX 3

namespace parser {
	// opcode definitions
	enum update_t {
		OP_NIL = 0,
		OP_POS = 1,
		OP_VEL = 2,
		OP_PV  = 3,
		OP_TUNE = 4,
		OP_BPM = 5
	};

	// Shared resources written by parser
	extern RawSerial pc;
	extern float target_pos;
	extern float target_vel;
	extern float tune_period; 	// this is the BPM/tempo, in case you were wondering
	extern update_t op_code;
	extern int tunes_list[TUNE_BUFFER]; 	// Buffer of periods, from parsed tune
	extern int durations_list[TUNE_BUFFER]; // Buffer of durations, from parsed tune
	extern volatile bool ready[4];

	// Parser functions
	extern void init();
	extern void pollSerialIn();
}

#endif