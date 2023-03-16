#include "SerialPlot.hpp"


SerialPlot::SerialPlot(BufferedSerial *pc, std::chrono::microseconds timeSample) {
    _pc = pc;
    _timeSample = timeSample;
    _number_variables = 0;
    std::string s = "Constructeur\n";
    _pc->write(s.c_str(), s.length());
}

SerialPlot::~SerialPlot() {
    debug_ticker.detach();
    debug_thread.terminate();
}


void SerialPlot::run() {
    debug_thread.start(callback(this, &SerialPlot::threadWorker));
}

void SerialPlot::tickerWorker() {
    switch (_number_variables) {
        case 1:
            sprintf(buffer, format,  *_variables[0]);
            break;
        case 2:
            sprintf(buffer, format,  *_variables[0], *_variables[1]);
            break;
        case 3:
            sprintf(buffer, format,  *_variables[0], *_variables[1], *_variables[2]);
            break;
        case 4:
            sprintf(buffer, format,  *_variables[0], *_variables[1], *_variables[2], *_variables[3]);
            break;
        case 5:
            sprintf(buffer, format, *_variables[0], *_variables[1], *_variables[2], *_variables[3], *_variables[4]);
            break;

    }
    _pc->write(buffer,sizeof(buffer));
}

void SerialPlot::threadWorker() {
    debug_ticker.attach(callback(this, &SerialPlot::tickerWorker), _timeSample);

    std::string s_deb = stringVariable(0)+"\t"+stringVariable(1)+"\n";
    _pc->write(s_deb.c_str(), s_deb.length());

    std::string s = "Ticker attaché\n";
    _pc->write(s.c_str(), s.length());
}

void SerialPlot::setVariables(volatile double *var1, volatile double *var2, volatile double *var3, volatile double *var4) {
    setVariables(var1);
    setVariables(var2);
    setVariables(var3);
    setVariables(var4);
}

void SerialPlot::setVariables(volatile double *var1, volatile double *var2, volatile double *var3) {
    setVariables(var1);
    setVariables(var2);
    setVariables(var3);
}

void SerialPlot::setVariables(volatile double *var1, volatile double *var2) {
    setVariables(var1);
    setVariables(var2);
}

void SerialPlot::setVariables(volatile double *var1) {
    _variables[_number_variables] = var1;
    _number_variables ++;

}

std::string SerialPlot::stringVariable(uint8_t number) {
    std::string s;
    if (number < _number_variables) {
       s = to_string(*_variables[number]);
    }
    else {
        s = "Error";
    }
    return s;
}

/* stringVariable(0)
++ MbedOS Error Info ++
Error Status: 0x80010133 Code: 307 Module: 1
Error Message: Mutex: 0x20001538, Not allowed in ISR context
Location: 0x8002323
Error Value: 0x20001538
Current Thread: rtx_idle Id: 0x20000FA8 Entry: 0x8002331 StackSize: 0x200 StackMem: 0x20000FF0 SP: 0x2001FE4C
For more info, visit: https://mbed.com/s/error?error=0x80010133&tgt=NUCLEO_F446RE
-- MbedOS Error Info --
 */