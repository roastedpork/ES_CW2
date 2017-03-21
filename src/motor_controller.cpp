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


    // PID variables
    static float error = 0;
    static float prev_error = 0;
    static float error_int = 0;
    static float error_delta = 0;

    static float pos_k = 100;
    static float pos_kp = 1;
    static float pos_kv = -0.75;
    static float pos_kd = 0.012;
    static float pos_ki = 0.025;

    static parser::update_t curr_op = parser::OP_NIL;


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
   //    curr_state = readRotorState();
   //    if ((timer.read_ms() < onPeriod) && (curr_state != prev_state)) {
   //            prev_state = curr_state;
   //            motorOut((curr_state-base_state+delta+6)%6); //+6 to make sure the remainder is positive
                   
   //    } else if (timer.read_ms() > period){
   //        timer.stop();
   //        timer.reset();
   //        timer.start();
   //    }
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
   //    float curr_angle = readRotorState()*60 + CH_ticks*TICK_RES;
   //    float error = target_angle - curr_angle;
   //    bool cycle_complete = false;
           
   //    float next_duty =  kp * error;
   //    delta = (next_duty > 0) ? 2 : -2;
   //    next_duty = std::abs(next_duty);
   //    high_period_us = ((next_duty > 1) ? 1 : next_duty)* period_us;
           
   //    pwm_clk.start();
   //    while(!cycle_complete){
   //        float stamp = pwm_clk.read_us();
   //        debug.printf("timestamp: %f\n\r", stamp);
               
   //        curr_state = readRotorState();
   //        if((stamp < high_period_us) && (curr_state != prev_state)){
   //            prev_state = curr_state;
   //            motorOut((curr_state-base_state+delta+6)%6); //+6 to make sure the remainder is positive
   //        } else if(stamp > period_us) {   
   //            cycle_complete = true;
   //        }
   //    }


   //    pwm_clk.stop();
   //    pwm_clk.reset();
   //    debug.printf("target: %4.2f, angle: %4.2f, next_duty: %f, high_period_us: %d\n\r", target_angle, curr_angle, next_duty, high_period_us);
   //  }
   // }







   static float vController(const float target_v, const float actual_v){
        float step = 0.2;
        error = target_v - actual_v;

        float res = (error > 0) ? duty_cycle - step : duty_cycle + step;
        res = (res > 100) ? 100 : res;
        res = (res < -100) ? -100 : res;

        return res;
   }

    static float pController(const float target_p, const float actual_p, 
    				const float actual_v, const float dt){

        prev_error = error;
        error = target_p - actual_p;
        error_delta = (error - prev_error) / dt;
        error_int += (error + prev_error) / 2 * dt;

        float n = pos_kp * error + pos_kv * actual_v; // pos_kd * error_delta; + pos_ki * error_int;
        float res = pos_k * n;

        res = (res > 100) ? 100 : res;
        res = (res < -100) ? -100 : res;

        return res;
    }

    static void pvController(const float target_p, const float actual_p, 
    				const float target_v, const float actual_v, const float dt){
    	
    	float v_duty = vController(actual_v, target_v);
    	float p_duty = pController(target_p, actual_p, actual_v, dt);

    	duty_cycle = (target_v - actual_v > 0) ? p_duty : v_duty;
    }

   static void reset(){

        error_int = 0;
   }


    void nextStep(){
      Timer control_loop;
      Timer pos_loop;

      control_loop.start();
      pos_loop.start();

        while (1) {

            parser::update_t new_op = parser::op_code;

            if ((new_op != parser::OP_NIL) || (new_op != parser::OP_TUNE) || (new_op != parser::OP_BPM)) {
               curr_op = new_op;
               reset();
               control_loop.reset();
               pos_loop.reset();
               new_op = parser::OP_NIL;
            } 

            switch (curr_op) {
               case parser::OP_VEL:
                  // Get input from other parts
                  float actual_v = odometer::velocity;
                  float target_v = parser::target_vel;

                  // Calculate next step
                  vController(actual_v, target_v);


                  break;

                case parser::OP_POS:
                    actual_v = odometer::velocity;                    
                    float actual_p = odometer::position;
                    float target_p = parser::target_pos;

                    float dt = pos_loop.read();
                    pos_loop.reset();

                    // Calculate next step
                    pController(target_p, actual_p, actual_v, dt);
                    break;

                case parser::OP_PV:
                    actual_v = odometer::velocity;                    
                    target_v = parser::target_vel;
                    actual_p = odometer::position;
                    target_p = parser::target_pos;

                    float dt = pos_loop.read();
                    pos_loop.reset();

                    // Calculate next step
                    pvController(target_p, actual_p, target_v, actual_v, dt);
                    break;
                    
                default:
                    break;
            }

            int readout = control_loop.read() * 1000;
            control_loop.reset();
            Thread::wait((readout < 20) ? (20 - readout) : 0);
                
        }
    }
}