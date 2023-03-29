// ============= INCLUDES =================
#include "mbed.h"
#include "BrushlessEirbot.hpp"
// ============= DEFINITIONS =================
DigitalOut led(LED1);
BufferedSerial pc(USBTX, USBRX, 115200);
BrushlessEirbot motorL(&pc, Left, 78);


int main() {
    led = 0;

    motorL.setDutyCycle(-0.5);
    ThisThread::sleep_for(2s);
    motorL.setDutyCycle(0.5);
    ThisThread::sleep_for(2s);
    motorL.setDutyCycle(0.);

    led = 1;
    while (true) {
        led = !led;
        ThisThread::sleep_for(1s);
    }
}
