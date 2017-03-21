#include "mbed.h"
#include "rtos.h"
#include "motor_controller.h"
#include "motor_driver.h"
#include "odometer.h"
#include "parser.h"

// Thread instantiations
Thread parser_thread(osPriorityNormal, 640);
Thread odometer_thread(osPriorityNormal, 300);
Thread controller_thread(osPriorityNormal, 600);
Thread driver_thread(osPriorityNormal, 400);

//Main
int main() {
    Serial pc(SERIAL_TX, SERIAL_RX);
    pc.baud(9600);

    parser::init();
    odometer::init();
    driver::init();

    parser_thread.start(parser::pollSerialIn);
    odometer_thread.start(odometer::updateState);
    controller_thread.start(controller::nextStep);    
    driver_thread.start(driver::runMotor);
    

    
    while(1){
       //  pc.printf("(Parser)     Total: %d, Free: %d, Used: %d(%d)\n\r", 
       //      parser_thread.stack_size(), parser_thread.free_stack(), 
       //      parser_thread.used_stack(), parser_thread.max_stack());
       //  pc.printf("(Odometer)   Total: %d, Free: %d, Used: %d(%d)\n\r", 
       //      odometer_thread.stack_size(), odometer_thread.free_stack(), 
       //      odometer_thread.used_stack(), odometer_thread.max_stack());
       //  pc.printf("(Controller) Total: %d, Free: %d, Used: %d(%d)\n\r", 
       //     controller_thread.stack_size(), controller_thread.free_stack(), 
       //     controller_thread.used_stack(), controller_thread.max_stack());
       //  pc.printf("(Driver)     Total: %d, Free: %d, Used: %d(%d)\n\r", 
       // driver_thread.stack_size(), driver_thread.free_stack(), 
       // driver_thread.used_stack(), driver_thread.max_stack());
        Thread::wait(1000);
        pc.printf("op: %d, target_p: %3.2f, actual_p: %3.2f, target_v: %3.2f, actual_v: %3.2f, duty: %3.2f \n\r", 
            parser::op_code, parser::target_pos, odometer::position, 
            parser::target_vel, odometer::velocity, controller::duty_cycle);
    }

    return 0;
}
