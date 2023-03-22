// ============= INCLUDES =================
#include "mbed.h"
#include "BrushlessEirbot.hpp"
#include "tim.h"
// ============= DEFINITIONS =================
DigitalOut led(LED1);
BufferedSerial pc(USBTX, USBRX, 115200);
BrushlessEirbot motorL(&pc, Left, 78);
BrushlessEirbot motorR(&pc, Right, 78);



int main() {
    led = 0;
    TIM1->CCR1 = 25;
    TIM8->CCR1 = 25;
    led = 1;
    while (true) {
        led = !led;
        ThisThread::sleep_for(1s);
    }
}
