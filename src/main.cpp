// ============= INCLUDES =================
#include "mbed.h"
#include "SerialPlot.hpp"
// ============= DEFINITIONS =================
volatile double i = 10.;
volatile double j = 30.;
double t = 0.;
BufferedSerial pc(USBTX, USBRX, 115200);

SerialPlot debugCom(&pc, 100ms);


// ============= FUNCTION =================

// ============= MAIN =================
int main() {
    std::string s = "Bienvenu !\n";
    pc.write(s.c_str(), s.length());
    debugCom.setVariables(&i, &j);
    debugCom.run();
    while (true) {
        i = sin(t);
        j = cos(t);
        t = t+0.1;
        ThisThread::sleep_for(1ms);
    }
}