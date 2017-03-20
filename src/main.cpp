#include "mbed.h"
#include "rtos.h"
#include "threaded_parser.h"

Thread parser_thread;
//Main
int main() {
    threaded_parser::init();

    parser_thread.start(threaded_parser::pollSerialIn);

    while(1){
        if (threaded_parser::output_ready){
            threaded_parser::pc.printf("(Main) pos: %f, vel: %f\n\r", threaded_parser::target_pos, threaded_parser::target_vel);
            threaded_parser::output_ready = false;
        }
    }

    return 0;
}


int main() {
    odometer::init();
    Serial pc(SERIAL_TX, SERIAL_RX);
    pc.baud(115200);
    parser_thread.start(odometer::updateState);
    Timer t;
    t.start();
    while(1){
        pc.printf("Current position: %f, update rate: %f\n\r", odometer::position, t.read()/odometer::update_count);
        Thread::wait(1000);
    }

    return 0;
}
