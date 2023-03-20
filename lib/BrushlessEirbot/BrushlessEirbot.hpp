#ifndef brushless_hpp
#define brushless_hpp

#include "mbed.h"
#include "tim.h"

#define P           8                 //p number of poles pairs
#define FREQ_TIM7   10000
#define MAX_SPEED   20000
#define MIN_SPEED   (-20000)
#define DC_MIN -90
#define DC_MAX 90

enum rotationSens_t {
    clockwise, antiClockwise
};
enum state {
    enable, busy
};
enum position {
    Left, Right
};
enum unitVelocity {
    rad_s, tick_s, mm_s
};

typedef struct {
    bool h1;
    bool h2;
    bool h3;
    char h123;
    char prev_h123;    //to determine sense of rotation

    int16_t tickS;    //to calculate speed
    int32_t tickP;    //tocallback(this, &BrushlessEirbot::hallInterrupt) calculate position
}hall_t;

typedef struct {
    // Channels enable bits
    bool aH;
    bool aL;
    bool bH;
    bool bL;
    bool cH;
    bool cL;
    rotationSens_t sens;
}PWM_t;

typedef struct{
    int32_t speed_ref;
    int32_t speed;

    int32_t error;
    int32_t prev_error;

    int32_t cmd;
    int32_t prev_cmd;
    int32_t cmdsat;
    int32_t prev_cmdsat;
}dataSpeed_t;

typedef struct {
    const float Kp;
    const float w_i;
    const float Te;
    // Integration coefficient : PI(z) = setSpeed_ref(z)/error(z) = Kp*(a0-a1*z^{-1})/(1-z^{-1})
    const float a0;
    const float a1;
}PID_t;


class BrushlessEirbot {
public:
    /**
     * Constructeur de l'object BrushlessEirbot avec l'information de la position du moteur. La position détermine le timer spécial utilisé.
     * Ils sont au nombre de 2 (timer1 et timer8)
     * Si la position est define sur *Left* le timer utilisé est le timer 1.
     * Si la position est define sur *Right* le timer utilisé est le timer 8.
     *
     *
     * @param position : Left ou Right selon le timer utilisé
     * @param wheelDiameterMm : Diamètre de la roue en millimètre
     */
    BrushlessEirbot(position position_motor, double wheelDiameterMm);
    /**
     * Constructeur permettant de renseigner un port série de débug.
     * @param pc : Port série de debug
     * @param position_motor : Left ou Right selon le timer utilisé
     * @param wheelDiameterMm : Diamètre de la roue en millimètre
     */
    BrushlessEirbot(BufferedSerial* pc, position position_motor, double wheelDiameterMm);
    void setVelocity(unitVelocity unit, double consigne);
    double getVelocity(unitVelocity unit) const;
    void displayPinOut();
    ~BrushlessEirbot();
private:
    state _stateController;
    Timer _timerVelocity;
    position _positionMotor;

    BufferedSerial* _serial;
    bool debug;

    void setDutyCycle();
    void decodeHall();
    void updateOutput();
    int16_t calculateSpeed();
    void writeCommand();

    hall_t hall;
    PWM_t PWM;
    dataSpeed_t data;

    /*
     *  Pins et instances
     */
    PinName _pinPWM_AH; PinName _pinPWM_AL;
    PinName _pinPWM_BH; PinName _pinPWM_BL;
    PinName _pinPWM_CH; PinName _pinPWM_CL;
    PinName _pinHall_1; PinName _pinHall_2; PinName _pinHall_3;
    PinName _pinCurrent_A; PinName _pinCurrent_B; PinName _pinCurrent_C;

    InterruptIn *HALL_1;
    InterruptIn *HALL_2;
    InterruptIn *HALL_3;
    void hallInterrupt();

    AnalogIn *Current_A;
    AnalogIn *Current_B;
    AnalogIn *Current_C;

    /*
     * Période d'échantillonnage pour les tickers.
     * TeUs_asserv : période en µs pour la fonction updateController() qui actualise la commande via le PID
     * TeUs_hall_secure : période en µs pour la lecture des Hall sensors, comme dans le cas où le moteur est très rapide,
     * il se peut qu'une interruption soit manquée, dans ce cas il y a un risque majeur de court-circuit sur le moteur /!\
     * Pour éviter cela nous allons mettre en place un processus périodique qui va regarder assez régulièrement l'état des Hall sensors
<    */
    const std::chrono::microseconds TeUsController = 10ms;
    const std::chrono::microseconds TeUsHallSecure = 100us;
    void _routineController();
    void _routineHallSecure();
    Ticker _tickerController;
    Ticker _tickerHallSecure;
    Ticker _hallSecure;
};

#endif