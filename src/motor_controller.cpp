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
    rtos::Mutex controller_mutex;

    // Position and velocity control variables
    static float error_p = 0;
    static float error_v = 0;
    static float error_int = 0;

    static const float pos_k = 75;
    static const float pos_kp = 0.75;
    static const float pos_kv = -1.2;

    static const float vel_k = 1.25;
    static const float vel_kp = 1.5;
    static const float vel_ki = 0.25;

    static parser::update_t curr_op = parser::OP_NIL;

    // Control functions which calculate the next duty cycle value
    static float vController(const float target_v, const float actual_v, const float dt){
        float prev_error = error_v;
        error_v = target_v - actual_v;
        error_int += (error_v + prev_error) / 2 * dt;

        float n = vel_kp * error_v + vel_ki * error_int; // pos_kd * error_delta; + pos_ki * error_int;
        float res = vel_k * n;
        res = (res > 100) ? 100 : res;
        res = (res < 0) ? 0 : res;
        return res;
    }

    static float pController(const float target_p, const float actual_p, const float actual_v){

        error_p = target_p - actual_p;

        float n = pos_kp * error_p + pos_kv * actual_v; 
        float res = pos_k * n;

        res = (res > 100) ? 100 : res;
        res = (res < -100) ? -100 : res;

        return res;
    }

    static float pvController(const float target_p, const float actual_p, const float target_v, const float actual_v, const float dt){
        float v_duty = vController(target_v, std::abs(actual_v), dt);
        float p_duty = pController(target_p, actual_p, actual_v);

//        v_duty = (target_p > 0) ? std::abs(v_duty) : -std::abs(v_duty);
        // yes, the math works out
        
        float max_abs_duty = (std::abs(v_duty) < std::abs(p_duty)) ? std::abs(v_duty) : std::abs(p_duty);
        
        if (max_abs_duty < std::abs(p_duty)){
        	return (p_duty > 0) ? max_abs_duty : - max_abs_duty;
        } else {
        	return p_duty;
        }

        //return (std::abs(v_duty) < std::abs(p_duty)) ? v_duty : p_duty;//(target_v - std::abs(actual_v) > 0) ? p_duty : v_duty;

//      return (target_v - std::abs(actual_v) > 0) ? p_duty : v_duty;
    }

    static void reset(){

        error_int = 0;
    }

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

        while (1) {
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

            odometer::odometer_mutex.lock();
            actual_p = odometer::position;
            actual_v = odometer::velocity;
            odometer::odometer_mutex.unlock();

            dt = pos_loop.read();
            pos_loop.reset();
            float res = 0;

            if(curr_op == parser::OP_VEL) {
                res = vController(target_v, actual_v, dt);
            
            } else if(curr_op == parser::OP_POS) {
                res = pController(target_p, actual_p, actual_v);
            
            } else if(curr_op == parser::OP_PV) {
                res = pvController(target_p, actual_p, target_v, actual_v, dt);
            
            }

            controller_mutex.lock();
            duty_cycle = res;
            controller_mutex.unlock();

            int readout = control_loop.read_ms();
            control_loop.reset();
            Thread::wait((readout < CTRL_PERIOD_MS) ? (CTRL_PERIOD_MS - readout) : 0);
                
        }
    }
}