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
    static PwmOut L1L(L1Lpin); 
    static PwmOut L1H(L1Hpin);
    static PwmOut L2L(L2Lpin);
    static PwmOut L2H(L2Hpin);
    static PwmOut L3L(L3Lpin);
    static PwmOut L3H(L3Hpin);

    // Variables for PWM controllers
    static int PWM_counter = 0;
    static Ticker pwm_ticker;
    static Ticker pwm_loop;
    static volatile bool update = false;
    static volatile bool do_pwm = false;


    // output for debugging
    int debug_int = 0; 
	float debug_f = 0;
	parser::update_t debug_op = parser::OP_NIL;
	parser::update_t debug_curr_op = parser::OP_NIL;
	parser::update_t debug_new_op = parser::OP_NIL;

    // state sync value
    static volatile int8_t base_state;  
    static volatile int8_t curr_state;
    static volatile int8_t prev_state;

    // Op code variable to handle input
    volatile parser::update_t curr_op = parser::OP_NIL;

    // Variables for tune playing
    static float beat_period = 0.5;
    
    //Set a given drive state
    static void motorOut(int8_t driveState, float pwm_value){
        
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
        if (driveOut & 0x01) L1L = pwm_value;
        if (driveOut & 0x02) L1H = 0;
        if (driveOut & 0x04) L2L = pwm_value;
        if (driveOut & 0x08) L2H = 0;
        if (driveOut & 0x10) L3L = pwm_value;
        if (driveOut & 0x20) L3H = 0;   
    }
        
    //Convert photointerrupter inputs to a rotor state
    static inline int8_t readRotorState(){

        return stateMap[odometer::I1 + 2*odometer::I2 + 4*odometer::I3];
    }

    static int8_t motorHome(){
        motorOut(0, 1);
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

    static void setPWMPeriod(float _p) {
		L1L.period(_p); 
		L1H.period(_p);
		L2L.period(_p);
		L2H.period(_p);
		L3L.period(_p);
		L3H.period(_p);
    }

    static void playTune(int half_period, float duration){
        float period_s = half_period / 500;
        setPWMPeriod(period_s);
        Timer beat;
        beat.start();

        // debug_int = half_period;
        // debug_f = duration;
        motorOut(0,0.03);
        while (beat.read() < duration) {
            // motorOut(0, 1);
            // Thread::wait(half_period);
            // motorOut(1, 1);
            // Thread::wait(half_period);
        }
    }

    static void lowSpeedPWM() {
        controller::controller_mutex.lock();
		float duty_cycle = controller::duty_cycle;
        controller::controller_mutex.unlock();

        int8_t delta = (duty_cycle > 0) ? 2 : -2;
    	duty_cycle = std::abs(duty_cycle)/ 100;

    	Timer slowpwm;
    	slowpwm.start();

    	while(slowpwm.read() < LOW_SPEED_PWM_PERIOD) {
			curr_state = readRotorState();

			if(curr_state != prev_state) {
				prev_state = curr_state;
				motorOut((curr_state - base_state + delta + 6) % 6, duty_cycle);
			}
		}
    }

    static void highSpeedPWM(){

        controller::controller_mutex.lock();
		float duty_cycle = controller::duty_cycle;
        controller::controller_mutex.unlock();
    
        int8_t delta = (duty_cycle > 0) ? 2 : -2;
    	duty_cycle = std::abs(duty_cycle);

        do_pwm = true;

        pwm_ticker.attach(&pwm_isr, PWM_TICK);
        pwm_loop.attach(&pwm_reset, PWM_PERIOD);
        // pwm_timer.start();

        while(do_pwm){
            curr_state = readRotorState();

            if((PWM_counter < duty_cycle) && (curr_state != prev_state)){
                prev_state = curr_state;
                motorOut((curr_state - base_state + delta + 6) % 6, 1);
            }
        }

        pwm_ticker.detach();
    }
    
    void runMotor() {
        Timer loop;
        volatile parser::update_t new_op = parser::OP_NIL;
        int read_tunes[TUNE_BUFFER];
        int read_durations[TUNE_BUFFER];
        debug_int = 0;
        debug_curr_op = parser::OP_NIL;
        debug_new_op = parser::OP_NIL;
        
        loop.start();
        while(1) {
            
            if (parser::ready[DRVR_INDEX]) {
                // Polls for any updates to the opcode and handles changes appropriately
                new_op = parser::op_code;

                if (new_op == parser::OP_TUNE){
                	std::memcpy(read_tunes, parser::tunes_list, TUNE_BUFFER * sizeof(int));
                	std::memcpy(read_durations, parser::durations_list, TUNE_BUFFER * sizeof(int));
                	beat_period = parser::tune_period;
                	
                } else if ((new_op == parser::OP_POS)  || (new_op == parser::OP_PV)) {
                	float target = parser::target_pos;
                	prev_state = (target > 0) ? ((curr_state - 1) % 6): ((curr_state + 1) % 6);
                
                } else if (new_op == parser::OP_VEL) {
                	prev_state = (curr_state - 1) % 6;
                }
                parser::ready[DRVR_INDEX] = false;

                setPWMPeriod(LOW_SPEED_PWM_PERIOD);
            }

			if (new_op != parser::OP_NIL) {
                curr_op = new_op;
            }

            // Executes stuff if new_op involves driving the motors
            if (curr_op == parser::OP_TUNE){
            		int8_t state = readRotorState();
            		if (state != base_state){
            			motorOut(0, 1);
            			Thread::wait(STALL_WAIT);
            		}
                    for (int i = 0; i < TUNE_BUFFER; i++) {
                        int half_period = read_tunes[i];
                        int reps = read_durations[i];
                        if (half_period) {
                        for (int j = 0; j < reps; j++){
	                            playTune(half_period, beat_period);// * 0.9);
	                            // Thread::wait(int(beat_period * 100)); // beat_period * 0.1 * 1000ms
                    		} 
                        }
                    }
            } else if (curr_op == parser::OP_NIL){
                Thread::wait(PWM_PERIOD);

            } else if ((curr_op == parser::OP_POS) || (curr_op == parser::OP_VEL) ||  (curr_op == parser::OP_PV)) {
                    odometer::odometer_mutex.lock();
                    float curr_v = odometer::velocity;
                    odometer::odometer_mutex.unlock();

                    if (curr_v < SPEED_THRESH){
                    	lowSpeedPWM();
                    } else {
	                    highSpeedPWM();

                    }
            } 

            // debug_new_op = new_op;
            // debug_curr_op = curr_op;

        }
    }

    void init(){

        base_state = motorHome();
        curr_state = base_state;
        prev_state = (base_state - 1) % 6;

		setPWMPeriod(LOW_SPEED_PWM_PERIOD);
    }
}