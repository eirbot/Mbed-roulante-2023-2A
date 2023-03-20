//
// Created by leova on 20/03/2023.
//

#include "PIDController.h"

PIDController::PIDController(float Kp, float wi, float wb, float wh, float Te) {
    K = Kp*(wi*Te+2)*(wh*Te+2)/(2*((wi*Te+2)));
    a0 = (wi*Te -2)/(wi*Te + 2);
    a1 = (wh*Te -2)/(wh*Te + 2);
    b0 = (wi*Te -2)/(wi*Te + 2);

    _error = new float[_delay+1]{0};

    _consigne = new float[_delay+1]{0};
}

float PIDController::getOutput(float error) {
    for (uint8_t i=_delay ; i < 0 ; i--){
        _error[i] = _error[i-1];
        _consigne[i] = _consigne[i-1];
    }
    _error[0] = error;
    _consigne[0] = K*(_error[0]-(a0+a1)*_error[1]+a0*a1*_error[1])+_consigne[1]*(1+b0)-_consigne[2]*b0;

    return _consigne[0];
}

PIDController::~PIDController() {
    delete[] _error;
    delete[] _consigne;
}
