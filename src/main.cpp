// ============= INCLUDES =================
#include "mbed.h"
#include "BrushlessEirbot.hpp"
#include "SerialPlot.hpp"
// ============= DEFINITIONS =================
DigitalOut led(LED1);

BufferedSerial pc(USBTX, USBRX, 115200);

SerialPlot debugPlot(&pc);
BrushlessEirbot motorL(&debugPlot, Left, 78);


int main() {
    led = 0;
//    motorL.setDutyCycle(0.4);
//    ThisThread::sleep_for(5s);
    motorL.setVelocity(tick_s, 100);
    ThisThread::sleep_for(5s);
    motorL.setVelocity(tick_s, 0);
//    motorL.setDutyCycle(0.5);
//    ThisThread::sleep_for(5s);

    led = 1;
    while (true) {
        led = !led;
        ThisThread::sleep_for(1s);
    }
}
