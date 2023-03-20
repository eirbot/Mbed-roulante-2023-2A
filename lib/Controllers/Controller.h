//
// Created by leova on 20/03/2023.
//

#ifndef MBED_CONTROLLER_H
#define MBED_CONTROLLER_H


class Controller {
public:
    virtual float getOutput(float error) = 0;
};


#endif //MBED_CONTROLLER_H
