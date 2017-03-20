#include <cmath>
#include "mbed.h"
#include "rtos.h"
#include "motor_controller.h"
#include "odometer.h"

namespace controller {
   // Serial output for debugging
   static Serial debug(SERIAL_TX, SERIAL_RX, 9600);

	// Variable output for motor driver
   	float duty_cycle = 50;


   	// PID variables
   	static float error = 0;
   	static float prev_error = 0;
   	static float error_int = 0;
   	static float error_delta = 0;

   	static float k_gain = 100;
   	static float k_p = 0.05;
   	static float k_i = 0.025;
   	static float k_d = 0.012;


   // Timer declarations
   


   // input specified number of rotations desired, rotates motor by said amount
   // void driveRotation(float _target) {
   //  int8_t delta = (_target > 0) ? 2 : -2;  //2 for forwards, -2 for backwards
   //  int8_t curr_state = 0;
   //  int8_t prev_state = 0;

   //  //Run the motor synchronisation
   //  base_state = motorHome();
   //  debug.printf("Rotor origin: %x\n\r",base_state);

   //  int totalLoopCycles = _target*6;
   //  int period = 10; 
   //  float dutyCycle = 0.8;
   //  int onPeriod = dutyCycle*period;

   //  timer.start();
	
   //  Timer speedometer;
   //  speedometer.start();
	
   //  while (deg60_ticks < totalLoopCycles) {
   //	  curr_state = readRotorState();
   //	  if ((timer.read_ms() < onPeriod) && (curr_state != prev_state)) {
   //			  prev_state = curr_state;
   //			  motorOut((curr_state-base_state+delta+6)%6); //+6 to make sure the remainder is positive
				   
   //	  } else if (timer.read_ms() > period){
   //		  timer.stop();
   //		  timer.reset();
   //		  timer.start();
   //	  }
   //  }   
	   
   //  speedometer.stop();
   //  float total_revs = deg60_ticks / 6.0;
   //  debug.printf("target:%4.2f , actual:%4.2f, speed:%4.2f\n\r", totalLoopCycles/6.0, total_revs, total_revs/speedometer.read());
   //  debug.printf("CH ticks: %d\n\r", CH_ticks);
   // }

   // void driveVelocity(float _target, float _vmax){
   //  Serial debug(SERIAL_TX, SERIAL_RX);
   //  int8_t delta = (_target > 0) ? 2 : -2;  //2 for forwards, -2 for backwards
   //  int8_t curr_state = 0;
   //  int8_t prev_state = 0;
	   
   //  float kp = 0.1;

   //  float target_angle = _target * 360;


   //  base_state = motorHome();
	   

   //  int period_us = 100000;
   //  float duty = 1.0;

   //  int high_period_us = period_us * duty; 


   //  Timer pwm_clk;



   //  while (1) {
   //	  float curr_angle = readRotorState()*60 + CH_ticks*TICK_RES;
   //	  float error = target_angle - curr_angle;
   //	  bool cycle_complete = false;
		   
   //	  float next_duty =  kp * error;
   //	  delta = (next_duty > 0) ? 2 : -2;
   //	  next_duty = std::abs(next_duty);
   //	  high_period_us = ((next_duty > 1) ? 1 : next_duty)* period_us;
		   
   //	  pwm_clk.start();
   //	  while(!cycle_complete){
   //		  float stamp = pwm_clk.read_us();
   //		  debug.printf("timestamp: %f\n\r", stamp);
			   
   //		  curr_state = readRotorState();
   //		  if((stamp < high_period_us) && (curr_state != prev_state)){
   //			  prev_state = curr_state;
   //			  motorOut((curr_state-base_state+delta+6)%6); //+6 to make sure the remainder is positive
   //		  } else if(stamp > period_us) {   
   //			  cycle_complete = true;
   //		  }
   //	  }


   //	  pwm_clk.stop();
   //	  pwm_clk.reset();
   //	  debug.printf("target: %4.2f, angle: %4.2f, next_duty: %f, high_period_us: %d\n\r", target_angle, curr_angle, next_duty, high_period_us);
   //  }
   // }







   static void velocityController(float target_v, float actual_v, float dt){
		int8_t delta = (target_v > 0) ? 2 : -2;

		// compute error, error
		error = target_v - actual_v;
		prev_error = error;

		error_delta = (error - prev_error) / dt;			// Gradient approximation
		error_int += ((error + prev_error) / 2) * dt;		// Trapezium approximation

		float d_del = kp*error + kd*der_error+ki*int_error;

		duty_cycle = gain * d_del;
		duty_cycle = (std::abs(duty_cycle) >= 100) ? 100 : -100;
   }

   // static void positionController(float _target, float _vmax){
   //	 int8_t delta = (_target > 0) ? 2 : -2;
   //	 int8_t curr_state = 0;
   //	 int8_t prev_state = 0;
   //	 float duty_cycle = 50;
   //	 float prev_error = 0;
   //	 float int_error = 0;
	   
   //	 pwm_ticker.attach(&pwm_isr, 0.0005);
   //	 pwm_loop.attach(&pwm_reset,0.05);
   //	 speedometer.start();
   //	 dt.start();
	   
   //	 while(1){
   //		 float curr_angle = deg60_ticks*60 + CH_ticks*TICK_RES;
   //		 curr_state = readRotorState();
   //		 if((PWM_counter < duty_cycle) && (curr_state != prev_state)){
   //			 prev_state = curr_state;
   //			 motorOut((curr_state-base_state+delta+6)%6);
   //		 }

   //		 if(update){
   //			 float gain = 0.00025;
   //			 float kp = 1;
   //			 float kd = 0;
   //			 float ki = 0;
   //			 float error = _target*360 - curr_angle;
   //			 float der_error = (error - prev_error)/dt.read();
			   
   //			 int_error += (error + prev_error)*dt.read()/2;
			   
   //			 float next_speed = kp*error + kd*der_error + ki*int_error;
   //			 duty_cycle = gain * next_speed;
   //			 delta = (next_speed > 0) ? 2 : -2;
   //			 pc.printf("current:%f, delta: %d,next duty: %3.2f\n\r",curr_angle/360,delta,  duty_cycle);
   //			 duty_cycle = (duty_cycle <= 0) ? std::abs(duty_cycle) : duty_cycle;
   //			 duty_cycle = (duty_cycle >= 100) ? 100 : duty_cycle;
   //			 update = false;
   //			 prev_error = error;
   //			 dt.reset();
			   
   //		 }
		   
		   
   //	 }
   // }

   static void reset(){
		float actual_v = odometer::velocity;
		float target_v = parser::target_vel;

		error = target_v - actual_v;
		prev_error = error;
		error_delta = 0;
		error_int = 0;
   	}


	void nextStep(){
		while (1) {

			while (1) {
				// Calculate next step
				velocityController(15);
				
			}
		}
	}

}