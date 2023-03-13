#include "SerialPlot.hpp"
#include "rtos.h"

SerialPlot::SerialPlot(BufferedSerial* pc, const double* variables[], uint8_t number_variables, std::chrono::microseconds timeSample) {
    _pc = pc;
    _variables = new const double*[2];
    _variables = variables;
    _number_variables = number_variables;
    _timeSample = timeSample;

    std::string s = "Constructeur\n";
    _pc->write(s.c_str(), s.length());
    s = to_string(*_variables[0]);
    s.append("\t"+to_string(*_variables[1]));
    s.append("\n");
    _pc->write(s.c_str(), s.length());
}

SerialPlot::~SerialPlot() {
    debug_ticker.detach();
    debug_thread.terminate();
    delete _variables;
}

void SerialPlot::run() {
    debug_thread.start(callback(this, &SerialPlot::threadWorker));
}

void SerialPlot::tickerWorker() {
//    std::string s = to_string(*_variables[0]);
//    if (_number_variables > 1) {
//        for (uint8_t i = 1; i < _number_variables; i++) {
//            s.append(",");
//            s.append(to_string(*_variables[i]));
//        }
//    }
//    s.append(",");
//    s.append(to_string(*_variables[1]));
//    s.append("\n");
    std::string s = "Ticker ";
//    s.append(to_string(*_variables[0]));
    s.append("\n");
    _pc->write(s.c_str(),s.length());
}

void SerialPlot::threadWorker() {
    debug_ticker.attach(callback(this, &SerialPlot::tickerWorker), _timeSample);

    std::string s = "Ticker attaché\n";
    _pc->write(s.c_str(), s.length());
}


