// ============= INCLUDES =================
#include "mbed.h"
#include "BrushlessEirbot.hpp"
// ============= DEFINITIONS =================

DigitalOut led(LED1);
BrushlessEirbot motorR(Right);

// ============= FUNCTION =================


// ============= MAIN =================
int main() {

    while (true) {
        cout << motorR.getTick() << endl;
        wait_us(1e5);

    }
}