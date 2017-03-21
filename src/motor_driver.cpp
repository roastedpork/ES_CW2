#include <cmath>
#include "mbed.h"
#include "rtos.h"
#include "motor_controller.h"
#include "motor_driver.h"
#include "odometer.h"
#include "parser.h"


namespace driver {

	//Drive state to output table
	static const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};
	static const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  

	//Digital Motor Drive outputs
	static DigitalOut L1L(L1Lpin); 
	static DigitalOut L1H(L1Hpin);
	static DigitalOut L2L(L2Lpin);
	static DigitalOut L2H(L2Hpin);
	static DigitalOut L3L(L3Lpin);
	static DigitalOut L3H(L3Hpin);

	// Variables for PWM controllers
	static int PWM_counter = 0;
	static Ticker pwm_ticker;
	static Ticker pwm_loop;
	static volatile bool update = false;
	static volatile bool do_pwm = false;


	// Serial output for debugging
	static Serial debug(SERIAL_TX, SERIAL_RX, 9600);	

	// state sync value
	static volatile int8_t base_state;  
	static volatile int8_t curr_state;
	static volatile int8_t prev_state;

	// Op code variable to handle input
	parser::update_t curr_op = OP_NIL;

	// Variables for tune playing
	static float beat_period = 0.5;
	
	//Set a given drive state
	static void motorOut(int8_t driveState){
		
		//Lookup the output byte from the drive state.
		int8_t driveOut = driveTable[driveState & 0x07];
		int8_t ndriveOut = ~driveOut; 
		
		// Normal Speed Setting
		//Turn off first
		if (ndriveOut & 0x01) L1L = 0;
		if (ndriveOut & 0x02) L1H = 1;
		if (ndriveOut & 0x04) L2L = 0;
		if (ndriveOut & 0x08) L2H = 1;
		if (ndriveOut & 0x10) L3L = 0;
		if (ndriveOut & 0x20) L3H = 1;
		
		//Then turn on
		if (driveOut & 0x01) L1L = 1;
		if (driveOut & 0x02) L1H = 0;
		if (driveOut & 0x04) L2L = 1;
		if (driveOut & 0x08) L2H = 0;
		if (driveOut & 0x10) L3L = 1;
		if (driveOut & 0x20) L3H = 0;   
	}
		
	//Convert photointerrupter inputs to a rotor state
	static inline int8_t readRotorState(){

		return stateMap[odometer::I1 + 2*odometer::I2 + 4*odometer::I3];
	}

	static int8_t motorHome(){
		motorOut(0);
		wait(1);
		return readRotorState();	
	}

	static void pwm_isr(){
		PWM_counter = (PWM_counter == 100) ? 100 : ++PWM_counter;
		pwm_ticker.attach(&pwm_isr, PWM_TICK);
	}

	static void pwm_reset(){
		PWM_counter = 0;
		do_pwm = false;
	}

	static void playTune(int half_period, float duration){
		Timer beat;
		beat.start();

		while (beat.read() < duration) {
			motorOut(prev_state);
			Thread::wait(half_period);
			motorOut(curr_state);
			Thread::wait(half_period);
		}
	}

	static void drivePWM(){
		//Timer pwm_timer;
		float duty_cycle = controller::duty_cycle;
		int8_t delta = (duty_cycle > 0) ? 2 : -2;

		duty_cycle = std::abs(duty_cycle);
		do_pwm = true;

		pwm_ticker.attach(&pwm_isr, PWM_TICK);
		pwm_loop.attach(&pwm_reset, PWM_PERIOD);
		// pwm_timer.start();

		while(do_pwm){
			//pwm_timer.reset();
			curr_state = readRotorState();

			if((PWM_counter < duty_cycle) && (curr_state != prev_state)){
				prev_state = curr_state;
				motorOut((curr_state-base_state+delta+6)%6);
			}

			// float readout = pwm_timer.read();
			// Thread::wait((readout < PWM_TICK) ? PWM_TICK - readout : 0);
		}

		pwm_ticker.detach();
	}
	
	void runMotor() {
		Timer loop;

		loop.start();
		while(1) {
			
			if (parser::ready[DRVR_INDEX]) {
				// Polls for any updates to the opcode and handles changes appropriately
				parser::update_t new_op = parser::op_code;
				parser::ready[DRVR_INDEX] = false;
			}

			if (new_op == parser::OP_BPM) {
			 	beat_period = parser::tune_period;

			} else if (new_op != parser::OP_NIL) {
			 	curr_op = new_op;

			}

			// Executes stuff if new_op involves driving the motors
			switch (curr_op) {
				case parser::OP_TUNE:
					//play tune
					for (int i = 0; i < TUNE_BUFFER; i++) {
						int half_period = parser::tunes_list[i];
						if (half_period) {
							playTune(half_period, beat_period * 0.9);
							Thread::wait(int(beat_period * 100)); // beat_period * 0.1 * 1000ms
						} else {
							Thread::wait(int(beat_period * 1000));
						}			 
					}

					break;

				//run PWM for 1 cycle
				default:
					drivePWM();

					float readout = loop.read();
					loop.reset();
					Thread::wait((readout < PWM_PERIOD) ? PWM_PERIOD - readout : 0);
					break;
			}

		}
	}

	void init(){

		base_state = motorHome();
		curr_state = base_state;
		prev_state = (base_state - 1) % 6;
	}
}