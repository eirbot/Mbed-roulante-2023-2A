// ============= INCLUDES =================
#include "mbed.h"
#include "SerialPlot.hpp"
// ============= DEFINITIONS =================
double i = 10.;
double j = 30.;
double t = 0.;
BufferedSerial pc(USBTX, USBRX, 115200);

#define NB 2
const double* variables[NB]={&i, &j};

SerialPlot debugCom(&pc, variables, NB, 100ms);

// ============= FUNCTION =================

// ============= MAIN =================
int main() {
    std::string s = "Bienvenu !\n";
    pc.write(s.c_str(), s.length());
    debugCom.run();
    while (true) {
        i = sin(t);
        j = cos(t);
        t = t+0.1;
        ThisThread::sleep_for(1ms);
    }
}