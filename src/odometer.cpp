#include "odometer.h"
#include "motor_driver.h"

namespace odometer {

	// Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
	// 0x00 and 0x07 are don't cares because no instance where all 3 photointerrupters are high or low 
	static const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  

	static const int8_t forwards[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x07, 0x07};

	// Photointerrupter input pins
	InterruptIn I1(I1pin);
	InterruptIn I2(I2pin);
	InterruptIn I3(I3pin);

	// Quadrature encoding input pins
	static InterruptIn CHAin(CHA);
	static DigitalIn CHBin(CHB);

	// Timer objects
	static Timer speedometer_CH;  

	// Tick variables for absolute position measurement
	static int deg60_ticks = 0; 
	static int CH_ticks = 0;
	static int current_motor_state = 0;
	static int prev_motor_state = 0;
	static volatile bool forwards_dir_hex = true;
	static volatile bool forwards_dir_CH = true;

	// Variables for velocity calculations
	static float velocity_CH = 0;
	static volatile bool update_speed = false;

	// Position and Velocity variables to be used by other parts of the code
	float position = 0;
	float velocity = 0;

	//Convert photointerrupter inputs to a rotor state
	static inline int8_t readRotorState(){
	
		return stateMap[I1 + 2*I2 + 4*I3];
	}
	
	// Input pins interrupt functions
	static void I_isr(){
		current_motor_state = readRotorState();

		if(forwards[prev_motor_state] == current_motor_state){
			deg60_ticks++;
			forwards_dir_hex = true;
		} else if (prev_motor_state != current_motor_state){
			deg60_ticks--;
			forwards_dir_hex = false;
		}

		CH_ticks = 0;
		prev_motor_state = current_motor_state;
	}

	static void CH_isr(){
		if(CHBin.read()){
			CH_ticks--;
			forwards_dir_CH = false;
			velocity_CH = -TICK_RES/speedometer_CH.read();
		} else {
			CH_ticks++;
			forwards_dir_CH = true;
			velocity_CH = TICK_RES/speedometer_CH.read();
		}

		update_speed = true;
		speedometer_CH.reset();
	}

	void updateState(){
		speedometer_CH.reset();

		speedometer_CH.start();

		float velocity_buffer[3]; 	// moving average buffer of 3 periods
		int ma = 0;					// moving average buffer index 

		while(1) {
			// Velocity update
			if (update_speed) {
				velocity_buffer[ma] = velocity_CH;
				update_speed = false;
			} else {
				velocity_buffer[ma] = 0;
			}

			ma = (ma + 1) % 3;
			velocity = (velocity_buffer[0] + velocity_buffer[1] + velocity_buffer[2]) / 3;

			// Position update
			if (forwards_dir_hex) {
				position = deg60_ticks * HEX_RES + CH_ticks * TICK_RES;
			} else {
				position = (deg60_ticks + 1) * HEX_RES + CH_ticks * TICK_RES;
			}

			Thread::wait(10);
		}
	}

	// Motor initialisation functions
	void init() {
		prev_motor_state = 0;//motor::motorHome();

		I1.rise(&I_isr);
		I1.fall(&I_isr);
		I2.rise(&I_isr);
		I2.fall(&I_isr);
		I3.rise(&I_isr);
		I3.fall(&I_isr); 
		CHAin.rise(&CH_isr);	   
	}   
}		