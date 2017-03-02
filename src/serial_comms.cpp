#include "serial_comms.h"
#include <cstdlib>

#define BUFF_SIZE 128

namespace serial_comms {

Serial pc(SERIAL_TX, SERIAL_RX);
float target_pos;
float target_vel;
bool update = false;

static char buffer[BUFF_SIZE];


static int parseCommand(const int _length){
	char op_char = 0;
	char float_buffer[8];
	
	int count = 0;
	int float_buffer_ind = 0;

	// Return error value
	if (_length <= 0){
		return 0;
	}
	while (count < _length) {
		char current_char = buffer[count++];
		switch(current_char){
			case 'R':
				if ((op_char == 'V') || (op_char == 'R')) {
					return 0;
				} else {
					op_char = current_char; 
				}
				break;

			case 'V':
				if (op_char == 'R') {
					op_char = 'V';
					target_pos = (float)std::atof(float_buffer);
					std::memset(float_buffer,0,8);
					float_buffer_ind = 0;
				} else if (op_char == 'V'){
					return 0;
				} else {
					op_char = current_char; 
				}
				break;
			
			case '\r':
				if(op_char == 'R'){
					target_pos = (float)std::atof(float_buffer);
				} else if (op_char == 'V') {
					target_vel = (float)std::atof(float_buffer);
				}
				
				break;

			default:
				float_buffer[float_buffer_ind++] = current_char;
				break;
		}
	}


	return 1;
}

static int pollSerialIn(){
	int counter = 0;
	bool done = false;
	
	std::memset(buffer, 0, BUFF_SIZE); 
	
	while(!done) {
		while(pc.readable()){
			buffer[counter++] = pc.getc();
			pc.putc(buffer[counter-1]);
			if (buffer[counter-1] == '\r') {			
				pc.printf("echo: %s\n", buffer); 
				done = true;
			}
		}
	}
	
	return counter; 
}

void getTargets(){
	int length;
	while(1){

		length = pollSerialIn();
		if(parseCommand(length)){
			update = true;
			pc.printf("target_pos: %f, target_vel %f\n\r",target_pos, target_vel);
		} else {
			update = false;
		}
	}

}

void init(){
	pc.baud(115200);
	// pc.attach(&receiveCallback);
}
} 