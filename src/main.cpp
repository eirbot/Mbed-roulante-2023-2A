// ================================= INCLUDES ================================
#include"mbed.h"
#include "BrushlessEirbot.hpp"
#include "SerialPlot.hpp"

// ========================== Variables Globales ==============================
DigitalOut led(LED1);
BufferedSerial pc(USBTX, USBRX, 115200);

BrushlessEirbot motorLeft(Left, 78);
BrushlessEirbot motorRight(Right, 78);

// ================================== DEBUG ==================================
double i;
double j;
#define numberVariables 2
double** SerialPlot::_variables = new double*[numberVariables]{&i, &j};
SerialPlot debugger(&pc,numberVariables, 10ms);

// ================================== MAIN ====================================
int main() {
    debugger.run();

    led.write(0);
    motorRight.displayPinOut();

    motorRight.setVelocity(tick_s, 700);

    led.write(1);
    while (true) {
        led = !led;
        ThisThread::sleep_for(1s);
    }
}