// ============= INCLUDES =================
#include "mbed.h"
#include "BrushlessEirbot.hpp"
// ============= DEFINITIONS =================
BufferedSerial pc(USBTX, USBRX, 115200);
DigitalOut led(LED1);
BrushlessEirbot motor(&pc,Left, 78);


// ============= MAIN =================
int main() {
    led.write(0);
//    motorRight.delamrd();
    TIM1->CCR1 = 25; TIM1->CCR2 = 25; TIM1->CCR3 = 25;
    led.write(1);

    while (true) {
        led = !led;
        ThisThread::sleep_for(1s);
    }

}