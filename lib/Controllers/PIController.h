//
// Created by leova on 20/03/2023.
//

#ifndef MBED_PICONTROLLER_H
#define MBED_PICONTROLLER_H
#include "Controller.h"
#include <cstdint>

class PIController : public Controller{
public:
    PIController(float Kp, float wi, float Te);
    ~PIController();
    float getOutput(float error) override;

protected:
    float K;
    float a0;

private:
    const uint8_t _delay = 2;
    float* _consigne;
    float* _error;
};



#endif //MBED_PICONTROLLER_H
