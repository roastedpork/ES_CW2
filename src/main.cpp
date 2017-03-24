#include "mbed.h"
#include "rtos.h"
#include "motor_controller.h"
#include "motor_driver.h"
#include "odometer.h"
#include "parser.h"

Thread parser_thread(osPriorityNormal, 850);
Thread odometer_thread(osPriorityNormal, 300);
Thread controller_thread(osPriorityNormal, 400);
Thread driver_thread(osPriorityNormal, 700);

//Main
int main() {
    RawSerial pc = parser::pc;
    pc.printf("\n\rES_CW2\n\r");
    pc.printf("Team Majulah\n\r");
    pc.printf("Group Members:\n\r");
    pc.printf("Benjamin Ng Zhi Long\n\r");
    pc.printf("Bingjie Chan\n\r");
    pc.printf("Jia Ming Ang\n\r");

    pc.printf("\n\rInitializing Code...\n\r");
    parser::init();
    odometer::init();
    driver::init();

    pc.printf("\n\rInitializing Threads...\n\r");
    parser_thread.start(parser::pollSerialIn);
    odometer_thread.start(odometer::updateState);
    controller_thread.start(controller::nextStep);  
    driver_thread.start(driver::runMotor);
    
    // Variables from parser 
    parser::update_t read_op;
    parser::update_t driver_op;
    float read_tp;
    float read_tv;
    float read_ap;
    float read_av;
    float read_duty;
    float read_tunep;
    int read_tunes[TUNE_BUFFER];


    pc.printf("\n\rMotor ready!\n\r");
    pc.printf("\n\rINSTRUCTIONS\n\r");
    pc.printf("===================================================\n\r");

    pc.printf("R<value> to set rotations\n\r");
    pc.printf("V<value> to set velocity\n\r");
    pc.printf("R<val1>V<val2> to set rotations with a max velocity\n\r");
    pc.printf("T(<note><duration>){1,16} to play a tune\n\r");
    pc.printf("M<value> to set tempo (BPM)\n\r");
    
    pc.printf("===================================================\n\r");
    pc.printf("A readout of the current position and velocity\n\r");
    pc.printf("will be displayed when the motor is set to rotate.\n\r");

    while(1){
        // pc.printf("    MAX / ALLOC \n\r");
        // pc.printf("(P) %3d / %3d   \n\r", parser_thread.max_stack(), parser_thread.stack_size());
        // pc.printf("(O) %3d / %3d   \n\r", odometer_thread.max_stack(), odometer_thread.stack_size());
        // pc.printf("(C) %3d / %3d   \n\r", controller_thread.max_stack(), controller_thread.stack_size());
        // pc.printf("(D) %3d / %3d   \n\r", driver_thread.max_stack(), driver_thread.stack_size());


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
                pc.printf("Playing a tune at %3.fBPM~\n\r", 60/read_tunep);

            } else if (read_op == parser::OP_BPM) {
                pc.printf("Set BPM to %3.2f\n\r", 60/read_tunep);

            } 
            
        }            
        


        if ((read_op == parser::OP_POS) ||(read_op == parser::OP_VEL) || (read_op == parser::OP_PV)){
            driver_op = read_op;
        } else if (read_op == parser::OP_TUNE) {
            driver_op = parser::OP_NIL;
        }

        if ((driver_op <= 3) && (driver_op >= 1)){
            read_ap = odometer::position;
            read_av = odometer::velocity;
            read_duty = controller::duty_cycle;
            pc.printf("P: %3.2f, V: %3.2f, D: %3.2f\n\r", read_ap, read_av, read_duty);
        }

        // pc.printf("(D) from parser: %d, op_new: %d, op_curr: %d, counter: %d\n\r", parser::op_code, driver::debug_new_op, driver::debug_curr_op, driver::debug_int);
        Thread::wait(1000);
    }

    return 0;
}
