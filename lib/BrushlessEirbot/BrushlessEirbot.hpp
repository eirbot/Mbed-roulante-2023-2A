#ifndef BRUSHLESS_BASE_ROULANTE_BRUSHLESSEIRBOT_H
#define BRUSHLESS_BASE_ROULANTE_BRUSHLESSEIRBOT_H

#include "BrushlessEirbot.hpp"

#include "Controller.h"
#include "PIDController.h"
#include "PIController.h"
#include "tim.h"

#include "mbed.h"

#define DutyCycleMAX 90
#define TickPerRevolution 48
#define Reductor 14

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
const halfBridge_t clockwiseSequence[6] = { //  101 -> 001 -> 011 -> 010 -> 110 -> 100
        {false,true, false,false,true, false}, // 0b001
        {true, false,false,true, false,false}, // 0b010
        {true, false,false,false,false,true},  // 0b011
        {false,true, false,false,true, false}, // 0b100
        {false,true, true, false,false,false}, // 0b101
        {false,false,false,true, true, false}, // 0b110
};
const halfBridge_t antiClockwiseSequence[6] = {
        {false,false,false, true,true, false}, // 0b001
        {false,true, true, false,false,false}, // 0b010
        {false,true, false,false,true, false}, // 0b011
        {true, false,false,false,false,true}, // 0b100
        {true, false,false,true, false,false}, // 0b101
        {false,false,true, false,false,true}, // 0b110
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

    void setPI(float Kp, float wi, ::chrono::microseconds Te = 10ms);
    void setPID(float Kp, float wi, float wb, float wh, ::chrono::microseconds TeUsController = 10ms);

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
    volatile int32_t _ticks;
    BufferedSerial *_serial;
    bool _debug{false};
    std::string buffer;
    char cbuffer[20]{0};
    const halfBridge_t halfBridgeZEROS = {0,0,0,0,0,0};

    TIM_TypeDef* _tim;
    volatile uint8_t _hallWord_previous;

    const float deltaTheta = 2*M_PI/(TickPerRevolution*Reductor);


    void halfBridgeApply(halfBridge_t halfBridgeConfig);

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

    Controller* controller = nullptr; //FIXME mettre un correcteur par défaut

    std::chrono::microseconds TeUsController = 10ms;
    Ticker _tickerController;

    void _routineController();
};


#endif //BRUSHLESS_BASE_ROULANTE_BRUSHLESSEIRBOT_H
