#ifndef MBED_SERIALPLOT_HPP
#define MBED_SERIALPLOT_HPP

#include "mbed.h"
#include "ostream"

class SerialPlot {
public:
    SerialPlot(BufferedSerial* pc, const double* variables[], uint8_t number_variables, std::chrono::microseconds timeSample = 10ms);

    void run();

    ~SerialPlot();
private:
    Ticker debug_ticker;
    void tickerWorker();

    Thread debug_thread;

    BufferedSerial *_pc;

    uint8_t _number_variables;
    std::chrono::microseconds _timeSample = 10ms;

    const double **_variables; // tableau de pointeur vers des doubles

    void threadWorker();

};


#endif //MBED_SERIALPLOT_HPP
