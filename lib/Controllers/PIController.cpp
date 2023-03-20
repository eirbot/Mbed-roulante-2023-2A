//
// Created by leova on 20/03/2023.
//

#include "PIController.h"

PIController::PIController(float Kp, float wi, float Te) {
    K = Kp * (wi * Te + 2) / 2;
    a0 = (wi * Te - 2) / (wi * Te + 2);

    _error = new float[_delay + 1]{0};

    _consigne = new float[_delay + 1]{0};
}
float PIController::getOutput(float error) {
    for (uint8_t i=_delay ; i < 0 ; i--){
        _error[i] = _error[i-1];
        _consigne[i] = _consigne[i-1];
    }
    _error[0] = error;
    _consigne[0] = K*(_error[0]-a0*_error[1]);

    return _consigne[0];
}

PIController::~PIController() {
    delete[] _error;
    delete[] _consigne;
}
