#include "mbed.h"
#include "rtos.h"
#include "motor_controller.h"
#include "motor_driver.h"
#include "odometer.h"
#include "parser.h"

Thread parser_thread(osPriorityNormal, 1024);
Thread odometer_thread(osPriorityNormal, 300);
Thread controller_thread(osPriorityNormal, 600);
Thread driver_thread(osPriorityNormal, 400);

//Main
int main() {
    RawSerial pc = parser::pc;

    parser::init();
    odometer::init();
    driver::init();

    parser_thread.start(parser::pollSerialIn);
    odometer_thread.start(odometer::updateState);
    controller_thread.start(controller::nextStep);  
    driver_thread.start(driver::runMotor);
    
    // Variables from parser 
    parser::update_t read_op;
    float read_tp;
    float read_tv;
<<<<<<< HEAD
    float read_ap;
    float read_av;
    float read_duty;
=======
>>>>>>> parent of 20ab9c2... more to do
    float read_tunep;
    int read_tunes[TUNE_BUFFER];


    
    while(1){
        if (parser::ready[MAIN_INDEX]){
            read_op = parser::op_code;
            read_tp = parser::target_pos;
            read_tv = parser::target_vel;
            read_tunep = parser::tune_period;

            for (int i = 0; i < TUNE_BUFFER; i++) {
                read_tunes[i] = parser::tunes_list[i];
            }

            parser::ready[MAIN_INDEX] = false;
            
            if (read_op == parser::OP_NIL) {
                pc.printf("Invalid Input\n\r");

            } else if (read_op == parser::OP_POS) {
                pc.printf("Target position set to %3.2f\n\r", read_tp);

            } else if (read_op == parser::OP_VEL) {
                pc.printf("Target velocity set to %3.2f\n\r", read_tv);

            } else if (read_op == parser::OP_PV) {  
                pc.printf("Target position set to %3.2f\n\r", read_tp);
                pc.printf("Target velocity set to %3.2f\n\r", read_tv);

            } else if (read_op == parser::OP_TUNE) {
                pc.printf("Playing a tune~\n\r");

            } else if (read_op == parser::OP_BPM) {
                pc.printf("Set BPM to %3.2f\n\r", 60/read_tunep);

            } 
            
        }            
<<<<<<< HEAD

        read_ap = odometer::position;
        read_av = odometer::velocity;
        read_duty = controller::duty_cycle;

        pc.printf("P: %3.2f, V: %3.2f, D: %3.2f\n\r", read_ap, read_av, read_duty);
=======
>>>>>>> parent of 20ab9c2... more to do
        Thread::wait(1000);
    }

    return 0;
}
