#include <cmath>
#include "mbed.h"
#include "rtos.h"
#include "motor_controller.h"
#include "odometer.h"
#include  "parser.h"

namespace controller {
    // Shared resources written by the controller
    float duty_cycle = 50;
    rtos::Mutex controller_mutex;

    // Position and velocity control variables
    static float error_p = 0;
    static float error_v = 0;
    static float error_int = 0;

    static float pos_k = 75;
    static float pos_kp = 0.75;
    static float pos_kv = -1.2;

    static float vel_k = 1.25;
    static float vel_kp = 1.5;
    static float vel_ki = 0.25;

    static parser::update_t curr_op = parser::OP_NIL;

    // Velocity Controller Implementation
    static float vController(const float target_v, const float actual_v, const float dt){
        // Calculate new error terms
        float prev_error = error_v;
        error_v = target_v - actual_v;
        error_int += (error_v + prev_error) / 2 * dt;

        // Calculate next duty cycle
        float n = vel_kp * error_v + vel_ki * error_int;
        float res = vel_k * n;

        // Saturation of duty cycle from 0 to 100
        res = (res > 100) ? 100 : res;
        res = (res < 0) ? 0 : res;
        return res;
    }

    // Position Controller Implementation
    static float pController(const float target_p, const float actual_p, const float actual_v){
    	// Calculate error term
        error_p = target_p - actual_p;

        // Calculate next duty_cycle
        float n = pos_kp * error_p + pos_kv * actual_v; 
        float res = pos_k * n;

        // Saturates result between -100 and 100
        res = (res > 100) ? 100 : res;
        res = (res < -100) ? -100 : res;

        return res;
    }

    // Position and Velocity Controller Implementation
    static float pvController(const float target_p, const float actual_p, const float target_v, const float actual_v, const float dt){
        // Get next duty cycle values from each controller 
        float v_duty = vController(target_v, std::abs(actual_v), dt);
        float p_duty = pController(target_p, actual_p, actual_v);

        // Bounds p_duty to v_duty
        float max_abs_duty = (std::abs(v_duty) < std::abs(p_duty)) ? std::abs(v_duty) : std::abs(p_duty);
        
        if (max_abs_duty < std::abs(p_duty)){
        	return (p_duty > 0) ? max_abs_duty : - max_abs_duty;
        }
    
    	return p_duty;
    }

    // Resets the integral error term for the velocity controller
    static void reset(){

        error_int = 0;
    }

    // Constantly updates the next duty cycle based on a specified period
    void nextStep(){
        Timer control_loop;
        Timer pos_loop;

        // NEED TO SET INITIAL VALUES
        float actual_p = 0;
        float actual_v = 0;
        float target_p = 0;
        float target_v = 0;
        float dt = 1;

        duty_cycle = 50;
        error_p = 0;
        error_v = 0;
        curr_op = parser::OP_NIL;
        control_loop.start();
        pos_loop.start();

        // Threading runs this continuously
        while (1) {
            
            // Checks for updates from parser and copies target values
            if (parser::ready[CTRL_INDEX]) {
                parser::update_t new_op = parser::op_code;
                if ((new_op == parser::OP_POS) || (new_op == parser::OP_VEL) || (new_op == parser::OP_PV)) {
                    curr_op = new_op;
                    
                    reset();
                    
                    new_op = parser::OP_NIL;
                    target_p = parser::target_pos;
                    target_v = parser::target_vel;
                } 
                parser::ready[CTRL_INDEX] = false;
            }

            // Actual state reading from odometer with mutex implementation
            odometer::odometer_mutex.lock();
            actual_p = odometer::position;
            actual_v = odometer::velocity;
            odometer::odometer_mutex.unlock();

            // Time delta for velocity controller integral term
            dt = pos_loop.read();
            pos_loop.reset();
            float res = 0;

            // current opcode defines which controller function to use 
            if(curr_op == parser::OP_VEL) {
                res = vController(target_v, actual_v, dt);
            
            } else if(curr_op == parser::OP_POS) {
                res = pController(target_p, actual_p, actual_v);
            
            } else if(curr_op == parser::OP_PV) {
                res = pvController(target_p, actual_p, target_v, actual_v, dt);
            
            }

            // Writes the next duty cycle, with mutex implementation
            controller_mutex.lock();
            duty_cycle = res;
            controller_mutex.unlock();

            // This should ensure that the control loops runs minimially for that
            // specified period, but it does not seem useful in retrospect
            int readout = control_loop.read_ms();
            control_loop.reset();
            Thread::wait((readout < CTRL_PERIOD_MS) ? (CTRL_PERIOD_MS - readout) : 0);
                
        }
    }
}