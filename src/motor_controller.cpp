#include <cmath>
#include "mbed.h"
#include "rtos.h"
#include "motor_controller.h"
#include "odometer.h"
#include  "parser.h"

namespace controller {
	// Serial output for debugging
	static Serial debug(SERIAL_TX, SERIAL_RX, 9600);

	// Variable output for motor driver
	float duty_cycle = 50;


	// Position and velocity control variables
	static float error = 0;
	static float prev_error = 0;
	static float error_int = 0;
	static float error_delta = 0;

	static float pos_k = 100;
	static float pos_kp = 1;
	static float pos_kv = -0.75;
	// static float pos_kd = 0.012;
	// static float pos_ki = 0.025;

	static parser::update_t curr_op = parser::OP_NIL;

	// Control functions which calculate the next duty cycle value
	static float vController(const float target_v, const float actual_v){
		float step = 0.2;
		error = target_v - actual_v;

		float res = (error > 0) ? duty_cycle - step : duty_cycle + step;
		res = (res > 100) ? 100 : res;
		res = (res < -100) ? -100 : res;

		return res;
	}

	static float pController(const float target_p, const float actual_p, const float actual_v, const float dt){

		// prev_error = error;
		error = target_p - actual_p;
		// error_delta = (error - prev_error) / dt;
		// error_int += (error + prev_error) / 2 * dt;

		float n = pos_kp * error + pos_kv * actual_v; // pos_kd * error_delta; + pos_ki * error_int;
		float res = pos_k * n;

		res = (res > 100) ? 100 : res;
		res = (res < -100) ? -100 : res;

		return res;
	}

	static float pvController(const float target_p, const float actual_p, const float target_v, const float actual_v, const float dt){
		
		float v_duty = vController(actual_v, target_v);
		float p_duty = pController(target_p, actual_p, actual_v, dt);

		return (target_v - actual_v > 0) ? p_duty : v_duty;
	}

	static void reset(){

		error_int = 0;
	}

	void nextStep(){
		Timer control_loop;
		Timer pos_loop;

		float target_v;
		float target_p;

		control_loop.start();
		pos_loop.start();

		while (1) {
			if (parser::ready[CTRL_INDEX]) {
				parser::update_t new_op = parser::op_code;
				if ((new_op == parser::OP_POS) || (new_op == parser::OP_VEL) || (new_op != parser::OP_PV)) {
					curr_op = new_op;
					
					reset();
					
					new_op = parser::OP_NIL;
					target_p = parser::target_pos;
					target_v = parser::target_vel;
				} 
				parser::ready[CTRL_INDEX] = false;
			}


			switch (curr_op) {
				case parser::OP_VEL:
					// Get input from other parts
					float actual_v = odometer::velocity;

					// Calculate next step
					float res = vController(actual_v, target_v);

					duty_cycle = res;
					break;

				case parser::OP_POS:
					float actual_p = odometer::position;
					actual_v = odometer::velocity;					

					float dt = pos_loop.read();
					pos_loop.reset();

					// Calculate next step
					res = pController(target_p, actual_p, actual_v, dt);

					duty_cycle = res;
					break;

				case parser::OP_PV:
					actual_v = odometer::velocity;					
					actual_p = odometer::position;

					dt = pos_loop.read();
					pos_loop.reset();

					// Calculate next step
					res = pvController(target_p, actual_p, target_v, actual_v, dt);

					duty_cycle = res;
					break;
					
				default:
					// Do nothing otherwise
					break;
			}

			int readout = control_loop.read() * 1000;
			control_loop.reset();
			Thread::wait((readout < 20) ? (20 - readout) : 0);
				
		}
	}
}