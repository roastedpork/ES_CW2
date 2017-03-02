#include "mbed.h"
#include "rtos.h"
#include "serial_comms.h"

//Photointerrupter input pins
#define I1pin D2
#define I2pin D3
#define I3pin D4

//Motor Drive output pins   //Mask in output byte
#define L1Lpin PA_6         //0x01
#define L1Hpin D6           //0x02
#define L2Lpin PA_7         //0x04
#define L2Hpin D8           //0x08
#define L3Lpin PA_4         //0x10
#define L3Hpin D10          //0x20

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};
//Mask to invert the outputs for high side transistors
const int8_t motorHmask = 0x2a;
//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
DigitalIn I1int(I1pin);
DigitalIn I2int(I2pin);
DigitalIn I3int(I3pin);

//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//Global state
int32_t outState = 0;   //Current drive state
int32_t inState = 0;    //Current rotor state
int32_t leadState = 0;  //Offset to calculate drive state from rotor state

//Set a given drive state
void motorOut(int8_t driveState){
    //First turn all phases off to prevent shoot-through
    L1H = 1;
    L2H = 1;
    L3H = 1;
    
    //Lookup the output byte from the drive state. Apply high side inversion mask
    int8_t driveOut = driveTable[driveState & 0x07] ^ motorHmask;
    
    //Apply the output byte to the pins
    L1L = driveOut & 0x01;
    L2L = driveOut & 0x04;
    L3L = driveOut & 0x10;
    L1H = driveOut & 0x02;
    L2H = driveOut & 0x08;
    L3H = driveOut & 0x20;
    }
    
inline int8_t readRotorState(){
    return stateMap[I1int + 2*I2int + 4*I3int];
    }

//Basic synchronisation routine    
void motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(1.0);
    
    //Get the rotor state
    inState = readRotorState();
    
    //Calculate the rotor-to-drive offset by adding two (modulo 6) the current rotor state
    leadState = (inState+2)%6;
}
    
Thread serial_thread;

//Main
int main() {
    //Initialise the serial port    
    serial_comms::init();
    serial_comms::pc.printf("Hello\n\r");

    serial_thread.start(serial_comms::getTargets);    
    
//    //Run the motor synchronisation
//    motorHome();
//    
//    //Poll the rotor state and set the motor outputs accordingly to spin the motor
//    while (1) {
//        motorOut(readRotorState()+leadState);
//        }
//}

    return 0;
}

