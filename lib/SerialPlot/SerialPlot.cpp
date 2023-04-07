#include "SerialPlot.hpp"

/* ************************************************
 *             Contructors & Destructor
 * ************************************************/
SerialPlot::SerialPlot(BufferedSerial *pc, EventFlags* flag) {
    _pc = pc;
    setFlag(flag);
}


SerialPlot::SerialPlot(BufferedSerial *pc) : SerialPlot(pc, nullptr){

}



SerialPlot::~SerialPlot() {
    debug_thread.terminate();
    for (uint8_t i = 0; i < floatVariables.size(); i++) {
        delete floatVariables[i];
        floatVariables[i] = 0;
    }
    for (uint8_t i = 0; i < uint16_tVariables.size(); i++) {
        delete uint16_tVariables[i];
        uint16_tVariables[i] = 0;
    }
    for (uint8_t i = 0; i < intVariables.size(); i++) {
        delete intVariables[i];
        intVariables[i] = 0;
    }
    delete _flag;
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

void SerialPlot::addVariables(int *ptrVariable) {
    intVariables.push_back(ptrVariable);
}

void SerialPlot::addVariables(uint16_t *ptrVariable) {
    uint16_tVariables.push_back(ptrVariable);
}

void SerialPlot::addVariables(float *ptrVariable) {
    floatVariables.push_back(ptrVariable);
}

void SerialPlot::setTimeSample(const chrono::microseconds &timeSample) {
    _timeSample = timeSample;
}

void SerialPlot::setFlag(EventFlags *flag) {
    _flag = flag;
}


