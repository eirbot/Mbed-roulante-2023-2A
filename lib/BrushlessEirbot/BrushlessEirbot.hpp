#ifndef brushless_hpp
#define brushless_hpp

#include "mbed.h"
#include "../Controllers/Controller.h"
#include "../Controllers/PIDController.h"
#include "../Controllers/PIController.h"
#include "tim.h"

void TIMER1_init();
void TIMER8_init();

typedef struct {
    bool aH;
    bool aL;
    bool bH;
    bool bL;
    bool cH;
    bool cL;
} halfBridge_t;
const halfBridge_t clockwiseSequence[6] = {
        {0,0,1,0,0,1}, // 0b001
        {1,0,0,1,0,0}, // 0b010
        {1,0,0,0,0,1}, // 0b011
        {0,1,0,0,1,0}, // 0b100
        {0,1,1,0,0,0}, // 0b101
        {0,0,0,1,1,0}, // 0b110
};
const halfBridge_t antiClockwiseSequence[6] = {
        {0,0,0,1,1,0}, // 0b001
        {0,1,1,0,0,0}, // 0b010
        {0,1,0,0,1,0}, // 0b011
        {1,0,0,0,0,1}, // 0b100
        {1,0,0,1,0,0}, // 0b101
        {0,0,1,0,0,1}, // 0b110
};

enum rotationSens_t {
    clockwise, antiClockwise
};
enum state {
    activated, desactivated, emergency
};
enum position {
    Left, Right
};
enum unitVelocity {
    rad_s, tick_s, mm_s
};

class BrushlessEirbot {
public:
    BrushlessEirbot(position position_motor, float wheelDiameterMm);

    BrushlessEirbot(BufferedSerial *pc, position position_motor, float wheelDiameterMm);

    ~BrushlessEirbot();

    void init();

    void setPI(float Kp, float wi, std::chrono::microseconds Te = 10ms);
    void setPID(float Kp, float wi, float wb, float wh, std::chrono::microseconds TeUsController = 10ms);

    void setVelocity(unitVelocity unit, float consigne);
    void setDutyCycle(float dutyCycle);
    void setController(state stateController);

    float getVelocity(unitVelocity unit) const;
private:
    state _stateController;
    Timer _timerVelocity;
    position _positionMotor;
    float _wheelDiameterMm;
    rotationSens_t _sens;
    int32_t _ticks;
    BufferedSerial *_serial;
    bool _debug{false};
    std::string buffer;
    char cbuffer[20]{0};

    PinName _pinHall_1;
    PinName _pinHall_2;
    PinName _pinHall_3;
    PinName _pinCurrent_A;
    PinName _pinCurrent_B;
    PinName _pinCurrent_C;

    InterruptIn *HALL_1;
    InterruptIn *HALL_2;
    InterruptIn *HALL_3;

    void hallInterrupt();

    AnalogIn *Current_A;
    AnalogIn *Current_B;
    AnalogIn *Current_C;

    Controller* controller;

    std::chrono::microseconds TeUsController = 10ms;
    Ticker _tickerController;

    void _routineController();
};


#endif
