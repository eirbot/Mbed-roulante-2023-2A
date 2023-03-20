#ifndef MBED_SERIALPLOT_HPP
#define MBED_SERIALPLOT_HPP

#include "mbed.h"

class SerialPlot {
public:
    SerialPlot(BufferedSerial* pc, uint8_t number_variables, std::chrono::microseconds timeSample = 10ms);
    void run();

    ~SerialPlot();

private:
    Ticker debug_ticker;
    char buffer[20]; // peu être améliorer
    const char * _format;
    void tickerWorker();
    Thread debug_thread;
    BufferedSerial *_pc;
    std::chrono::microseconds _timeSample = 10ms;
    void threadWorker();
    uint8_t _number_variables;
    static double** _variables; // tableau de pointeur vers des doubles déclarer dans le main
};


#endif //MBED_SERIALPLOT_HPP
