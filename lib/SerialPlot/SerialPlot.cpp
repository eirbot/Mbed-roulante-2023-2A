#include "SerialPlot.hpp"

SerialPlot::SerialPlot(BufferedSerial *pc, uint8_t number_variables, std::chrono::microseconds time_sample) {
    _pc = pc;
    _number_variables = number_variables;
    _timeSample = time_sample;
    for (uint8_t i=0 ; i<20 ; i++) { buffer[i] = 0; }
    std::string s;
    for (uint8_t i=0 ; i<number_variables-1 ; i++){
        s.append("%f,");
    }
    s.append("%f\n");
    _format = new char[s.length()];
    _format = s.c_str();

//    std::string s = "Constructeur\n";
//    _pc->write(s.c_str(), s.length());
}

SerialPlot::~SerialPlot() {
    debug_ticker.detach();
    debug_thread.terminate();
    delete[] _format;
    delete[] _variables;
}

void SerialPlot::run() {
    debug_thread.start(callback(this, &SerialPlot::threadWorker));
}

void SerialPlot::tickerWorker() {
    for (uint8_t i = 0 ; i<_number_variables ; i++) {
        sprintf(buffer, _format, *_variables[i]);
        _pc->write(buffer, sizeof(buffer));
    }
}

void SerialPlot::threadWorker() {
    debug_ticker.attach(callback(this, &SerialPlot::tickerWorker), _timeSample);

//    std::string s = "Ticker attaché\n";
//    _pc->write(s.c_str(), s.length());
}