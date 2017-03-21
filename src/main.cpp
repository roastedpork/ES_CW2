#include "mbed.h"
#include "rtos.h"
#include "motor_controller.h"
#include "motor_driver.h"
#include "odometer.h"
#include "parser.h"

Thread controller_thread(osPriorityNormal, 768);
Thread odometer_thread(osPriorityNormal, 512);
Thread driver_thread(osPriorityNormal, 1024);
Thread parser_thread(osPriorityNormal, 640);
//Main
int main() {
    Serial pc(SERIAL_TX, SERIAL_RX);
    pc.baud(9600);

    parser::init();
    odometer::init();
    driver::init();

    parser_thread.start(parser::pollSerialIn);
    odometer_thread.start(odometer::updateState);
    driver_thread.start(driver::runMotor);
    controller_thread.start(controller::nextStep);    

    
    while(1){
        // pc.printf("(Driver)   Total: %d, Using: %d, Free: %d\n\r", driver_thread.stack_size(), driver_thread.free_stack(), driver_thread.used_stack());
        // pc.printf("(Odometer) Total: %d, Using: %d, Free: %d\n\r", odometer_thread.stack_size(), odometer_thread.free_stack(), odometer_thread.used_stack());
        Thread::wait(100);
        pc.printf("Velocity %f \n\r", odometer::velocity);
    }

    return 0;
}
