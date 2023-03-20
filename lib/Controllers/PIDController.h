//
// Created by leova on 20/03/2023.
//

#ifndef MBED_PIDCONTROLLER_H
#define MBED_PIDCONTROLLER_H

#include "Controller.h"
#include <cstdint>

class PIDController : public Controller{
public:
    PIDController(float Kp, float wi, float wb, float wh, float Te);
    ~PIDController();
    float getOutput(float error) override;
protected:
    float K;
    float a0;
    float a1;
    float b0;
private:
    const uint8_t _delay = 3;
    float* _error;
    float* _consigne;
};



#endif //MBED_PIDCONTROLLER_H
