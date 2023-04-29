#include "SerialPlot.hpp"

/* ************************************************
 *             Contructors & Destructor
 * ************************************************/
SerialPlot::SerialPlot(BufferedSerial *pc, EventFlags* flag) {
    _pc = pc;
    _flag = flag;
}

SerialPlot::~SerialPlot() {
    debug_thread.terminate();
}

/* ************************************************
 *                  Public Methods
 * ************************************************/

void SerialPlot::run() {
    debug_thread.start(callback(this, &SerialPlot::threadWorker));
}

/* ************************************************
 *                  Private Methods
 * ************************************************/

void SerialPlot::Worker() {
    std::string str;
    for (uint8_t i = 0; i < (floatVariables.size() - 1) ; i++) {
        str.append(to_string(*floatVariables[i]));
        str.append(",");
    }
    str.append(to_string(*floatVariables[floatVariables.size() - 1]));
    str.append("\n");
    _pc->write(str.c_str(), str.length());
}

void SerialPlot::threadWorker() {
    while (true){
        _flag->wait_any(_flag->get());
        Worker();
    }
}

void SerialPlot::addVariables(float *ptrVariable) {
    floatVariables.push_back(ptrVariable);
}


