//#ifndef brushless_hpp
//#define brushless_hpp
//
//#include "mbed.h"
//#include "tim.h"
//
//typedef struct {
//    bool h1;
//    bool h2;
//    bool h3;
//    char h123;
//    char prev_h123;    //to determine sense of rotation
//
//    int16_t tickS;    //to calculate speed
//    int32_t tickP;    //to calculate position
//}hall_t;
//
//typedef struct {
//    // Channels enable bits
//    bool aH;
//    bool aL;
//    bool bH;
//    bool bL;
//    bool cH;
//    bool cL;
//    bool sense; //0=motor turns anti-clockwise 1=motor turns clockwise
//}PWM_t;
//
//typedef struct{
//    int16_t speed_ref;
//    int16_t speed;
//
//    int16_t error;
//    int16_t prev_error;
//
//    int16_t cmd;
//    int16_t prev_cmd;
//    int16_t cmdsat;
//    int16_t prev_cmdsat;
//}dataSpeed_t;
//
//typedef struct {
//    const float Kp;
//    const float w_i;
//    const float Te;
//    // Integration coefficient : PI(z) = setSpeed_ref(z)/error(z) = Kp*(a0-a1*z^{-1})/(1-z^{-1})
//    const float a0;
//    const float a1;
//}PID_t;
//
//
//// Enumeration pour les types un peu spéciaux
//enum half_bridge_t {
//    a, b, c
//};
//enum state_t {
//    enable, busy
//};
//enum position_t {
//    Left, Right
//};
//
//class BrushlessEirbot {
//public:
//    // Constructeurs & Destructeur
//    BrushlessEirbot(position_t position_motor);
//
//    ~BrushlessEirbot();
////
////    position_t position_motor;
////
//    void setDutyCycle();
////
////    void decodeHall(hall_t *hall, PWM_t *PWM);
////
////    void updateOutput(PWM_t PWM);
////
////    int16_t calculateSpeed(hall_t *hall);
////
////    void writeCommand(PWM_t *PWM, dataSpeed_t *data);
//
//    // Attributs
//    state_t state;
//    Timer timer_velocity_ms;
//
//private:
//    // Code de Vincent :
//    #define P           8                 //p number of poles pairs
//    #define FREQ_TIM7   10000
//    #define MAX_SPEED   20000
//    #define MIN_SPEED   -20000
//    #define DC_MIN -90
//    #define DC_MAX 90
//
//    position_t position;
//
//    hall_t hall;
//    PWM_t PWM;
//    dataSpeed_t data;
//
////    // Pins et instance
////    PinName pin_PWM_A_H;
////    PinName pin_PWM_A_L;
////    PinName pin_PWM_B_H;
////    PinName pin_PWM_B_L;
////    PinName pin_PWM_C_H;
////    PinName pin_PWM_C_L;
////    PinName pin_HALL_1;
////    PinName pin_HALL_2;
////    PinName pin_HALL_3;
////    PinName pin_Current_A;
////    PinName pin_Current_B;
////    PinName pin_Current_C;
////
////    InterruptIn *HALL_1;
////    InterruptIn *HALL_2{};
////    InterruptIn *HALL_3{};
////    AnalogIn *Current_A;
////    AnalogIn *Current_B{};
////    AnalogIn *Current_C{};
//
//    /*
//     * Période d'échantillonnage pour les tickers.
//     * TeUs_asserv : période en µs pour la fonction updateController() qui actualise la commande via le PID
//     * TeUs_hall_secure : période en µs pour la lecture des Hall sensors, comme dans le cas où le moteur est très rapide,
//     * il se peut qu'une interruption soit manquée, dans ce cas il y a un risque majeur de court-circuit sur le moteur /!\
//     * Pour éviter cela nous allons mettre en place un processus périodique qui va regarder assez régulièrement l'état des Hall sensors
//     *
//<     */
//    const std::chrono::microseconds TeUS_asserv = 10ms; //attaché à un ticker_us
//    const std::chrono::microseconds TeUs_hall_secure = 10ms;
//};
//
//#endif