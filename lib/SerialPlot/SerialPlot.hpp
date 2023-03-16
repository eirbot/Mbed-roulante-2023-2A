#ifndef MBED_SERIALPLOT_HPP
#define MBED_SERIALPLOT_HPP

#include "mbed.h"


class SerialPlot {
public:
    SerialPlot(BufferedSerial* pc, std::chrono::microseconds timeSample = 10ms);
    void setVariables(volatile double *var1, volatile double *var2, volatile double *var3, volatile double *var4, volatile double *var5);
    void setVariables(volatile double *var1, volatile double *var2, volatile double *var3, volatile double *var4);
    void setVariables(volatile double *var1, volatile double *var2, volatile double *var3);
    void setVariables(volatile double *var1, volatile double *var2);
    void setVariables(volatile double *var1);
    void run();

    ~SerialPlot();

private:
    std::string stringVariable(uint8_t number);
    Ticker debug_ticker;
    char buffer[50];
    const char *format;
    void tickerWorker();
    Thread debug_thread;
    BufferedSerial *_pc;
    uint8_t _number_variables;
    std::chrono::microseconds _timeSample = 10ms;
    void threadWorker();
    volatile double *_variables[5]; // tableau de pointeur vers des doubles
};


#endif //MBED_SERIALPLOT_HPP
