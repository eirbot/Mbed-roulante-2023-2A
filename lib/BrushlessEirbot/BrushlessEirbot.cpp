#include "BrushlessEirbot.hpp"

#define DutyCylcleMAX        90
#define ticksPerRevolution   (14*48)


void TIMER1_init(){
    MX_TIM1_Init();

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}
void TIMER8_init(){
    MX_TIM8_Init();

    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
}


/* *******************************************************************************
 *                          Constructeur et destructeurs
 * *******************************************************************************/

BrushlessEirbot::BrushlessEirbot(position position_motor, float wheelDiameterMm) {
    if (!_debug){
        delete _serial; // pas de debug
    }

    _positionMotor = position_motor;
    _stateController = activated;
    _wheelDiameterMm = wheelDiameterMm;
    _sens = clockwise;
    _ticks = 0;

    _tickerController.attach(callback(this, &BrushlessEirbot::_routineController), TeUsController);

    if (_positionMotor == Left) {
        // Configuration des Hall Sensors
        _pinHall_1 = PB_2;
        _pinHall_2 = PB_5;
        _pinHall_3 = PB_6;

        // Configuration des Current Sensors
        _pinCurrent_A = PC_0;
        _pinCurrent_B = PC_1;
        _pinCurrent_C = PC_2;
#ifndef timerPWM
#define timerPWM TIM1
#endif
        TIMER1_init();
    }
    else if (_positionMotor == Right) {
        // Configuration des Hall Sensors
        _pinHall_1 = PB_7;
        _pinHall_2 = PB_8;
        _pinHall_3 = PB_9;

        // Configuration des Current Sensors
        _pinCurrent_A = PC_3;
        _pinCurrent_B = PC_4;
        _pinCurrent_C = PC_5;
#ifndef timerPWM
#define timerPWM TIM8
#endif
        TIMER8_init();
    }

    HALL_1 = new InterruptIn(_pinHall_1, PullNone);
    HALL_2 = new InterruptIn(_pinHall_2, PullNone);
    HALL_3 = new InterruptIn(_pinHall_3, PullNone);
    HALL_1->rise(callback(this, &BrushlessEirbot::hallInterrupt));
    HALL_2->rise(callback(this, &BrushlessEirbot::hallInterrupt));
    HALL_3->rise(callback(this, &BrushlessEirbot::hallInterrupt));

    Current_A = new AnalogIn(_pinCurrent_A);
    Current_B = new AnalogIn(_pinCurrent_B);
    Current_C = new AnalogIn(_pinCurrent_C);
}

BrushlessEirbot::BrushlessEirbot(BufferedSerial *pc, position position_motor, float wheelDiameterMm): BrushlessEirbot(position_motor, wheelDiameterMm) {
    _debug = true;
    _serial = pc;
    buffer = "Constructeur ";
    _serial->write(buffer.c_str(), buffer.length());

}

BrushlessEirbot::~BrushlessEirbot() {
    _tickerController.detach();
    _tickerController.detach();
    delete HALL_1;
    delete HALL_2;
    delete HALL_3;
    delete Current_A;
    delete Current_B;
    delete Current_C;
    if(_debug) {
        delete _serial;
    }
}

void BrushlessEirbot::init(){
//    if (_positionMotor == Left) {
//#ifndef timerPWM
//#define timerPWM TIM1
//#endif
//        TIMER1_init(); // Le init doit être défini en "dur" ou en méthode static
//
//    }
//    else if (_positionMotor == Right) {
//#ifndef timerPWM
//#define timerPWM TIM8
//#endif
//        TIMER8_init(); // Le init doit être défini en "dur" ou en méthode static
//    }
}

/* *******************************************************************************
 *                              Méthodes publiques
 * *******************************************************************************/
void BrushlessEirbot::setVelocity(unitVelocity unit, float consigne) {
    //TODO Mettre en forme le controller et les conversions
    _stateController = activated;
//    switch (unit) {
//        case rad_s:
//            break;
//        case tick_s:
//            break;
//        case mm_s:
//            break;
//    }
}

void BrushlessEirbot::setController(state stateController){
    _stateController = stateController;
}

float BrushlessEirbot::getVelocity(unitVelocity unit) const {
    // TODO Fixé le timer pour résoudre le problème temporel
    switch (unit) {
        case rad_s:
            return _ticks*M_2_PI/ticksPerRevolution; // convert to rad/s
        case tick_s:
            return _ticks; // convert to tick/s
        case mm_s:
            return _ticks*M_2_PI*_wheelDiameterMm; // convert to mm/s
            break;
        default:
            return (MAXFLOAT);
    }
}

void BrushlessEirbot::setPI(float Kp, float wi, std::chrono::microseconds Te_chrono) {
    TeUsController = Te_chrono;
    float Te = (float)TeUsController.count() / float(1e-6);
    controller = new PIController(Kp, wi, Te);
}

void BrushlessEirbot::setPID(float Kp, float wi, float wb, float wh, std::chrono::microseconds Te_chrono) {
    TeUsController = Te_chrono;
    float Te =  (float) TeUsController.count() / float(1e-6);
    controller = new PIDController(Kp, wi, wb, wh, Te);
}


void BrushlessEirbot::setDutyCycle(float dutyCycle) {
    // Configuration du sens de rotation
    if (dutyCycle >= 0) {_sens = clockwise;}
    else                {_sens = antiClockwise;}

    // Conversion vers du uint8_t
    uint8_t dutyCycle_int = (uint8_t) (fabs(dutyCycle)*100); //FIXME Augmenter la résolution du comparateur (penser à changer prescaler)

    buffer = "dutyCycle"+ to_string(dutyCycle_int)+"\n";
    _serial->write(buffer.c_str(), buffer.length());

    // Saturation
    if (dutyCycle_int >= DutyCylcleMAX){dutyCycle_int = DutyCylcleMAX;}

    timerPWM->CCR1 = dutyCycle_int;
    timerPWM->CCR2 = dutyCycle_int;
    timerPWM->CCR3 = dutyCycle_int;
}

/* *******************************************************************************
 *                     Cadencement des méthodes primaires et asservissement
 * *******************************************************************************/
void BrushlessEirbot::hallInterrupt() {
    // Lecture Hall sensors
    uint8_t halls = (HALL_1->read()<<2) | (HALL_2->read()<<1) | HALL_3->read();

    // Affectation de la séquence d'après
    halfBridge_t halfBridge;
    if (_sens == clockwise){
        halfBridge = clockwiseSequence[halls-1];
        _ticks++; // Indentation des ticks
    }
    else{
        halfBridge = antiClockwiseSequence[halls-1];
        _ticks--; // Indentation des ticks
    }

    // Mise en oeuvre des commandes
    if (halfBridge.aH) {                       //CC1E=1
        timerPWM->CCER |= TIM_CCER_CC1E;
    } else {                            //CC1E=0
        timerPWM->CCER &= ~(TIM_CCER_CC1E);
    }

    if (halfBridge.aL) {                       //CC1NE=1
        timerPWM->CCER |= TIM_CCER_CC1NE;
    } else {                            //CC1NE=0
        timerPWM->CCER &= ~(TIM_CCER_CC1NE);
    }

    if (halfBridge.bH) {
        timerPWM->CCER |= TIM_CCER_CC2E;
    } else {
        timerPWM->CCER &= ~(TIM_CCER_CC2E);
    }

    if (halfBridge.bL) {
        timerPWM->CCER |= TIM_CCER_CC2NE;
    } else {
        timerPWM->CCER &= ~(TIM_CCER_CC2NE);
    }

    if (halfBridge.cH) {
        timerPWM->CCER |= TIM_CCER_CC3E;
    } else {
        timerPWM->CCER &= ~(TIM_CCER_CC3E);
    }

    if (halfBridge.cL) {
        timerPWM->CCER |= TIM_CCER_CC3NE;
    } else {
        timerPWM->CCER &= ~(TIM_CCER_CC3NE);
    }
}

void BrushlessEirbot::_routineController() {

}