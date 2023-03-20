#include "BrushlessEirbot.hpp"

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

    // Initialisation des structures
    hall = {.h1=0, .h2=0, .h3=0, .h123=0, .prev_h123=0, .ticks=0};
    PWM = {.aH=1, .aL=0, .bH=1, .bL=0, .cH=0, .cL=0, .sens=clockwise};

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

        // Configuration du timer 1
        MX_TIM1_Init();
        #ifndef timerPWM
        #define timerPWM TIM1
        #endif
        // Configuration des Current Sensors
        _pinPWM_AH = PC_6;
        _pinPWM_AL = PA_7;
        _pinPWM_BH = PC_7;
        _pinPWM_BL = PB_14;
        _pinPWM_CH = PC_8;
        _pinPWM_CL = PB_15;
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

        // Configuration du timer 8
        MX_TIM8_Init();
        #ifndef timerPWM
        #define timerPWM TIM8
        #endif
    }

    HALL_1 = new InterruptIn(_pinHall_1, PullNone);
    HALL_2 = new InterruptIn(_pinHall_2, PullNone);
    HALL_3 = new InterruptIn(_pinHall_3, PullNone);
    HALL_1->rise(callback(this, &BrushlessEirbot::hallInterrupt));
    HALL_2->rise(callback(this, &BrushlessEirbot::hallInterrupt));
    HALL_3->rise(callback(this, &BrushlessEirbot::hallInterrupt));
    HALL_1->fall(callback(this, &BrushlessEirbot::hallInterrupt));
    HALL_2->fall(callback(this, &BrushlessEirbot::hallInterrupt));
    HALL_3->fall(callback(this, &BrushlessEirbot::hallInterrupt));

    Current_A = new AnalogIn(_pinCurrent_A);
    Current_B = new AnalogIn(_pinCurrent_B);
    Current_C = new AnalogIn(_pinCurrent_C);
}

BrushlessEirbot::BrushlessEirbot(BufferedSerial *pc, position position_motor, float wheelDiameterMm): BrushlessEirbot(position_motor, wheelDiameterMm) {
    _debug = true;
    _serial = pc;
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
    delete _serial;
    delete controller;
}

/* *******************************************************************************
 *                              Méthodes publiques
 * *******************************************************************************/

void BrushlessEirbot::setVelocity(unitVelocity unit, float consigne) {
    _stateController = activated;
    switch (unit) {
        case rad_s:
            break;
        case tick_s:
            break;
        case mm_s:
            break;
    }
}

void BrushlessEirbot::setController(state stateController){

}


double BrushlessEirbot::getVelocity(unitVelocity unit) const {
    switch (unit) {
        case rad_s:
            return hall.ticks*(2*M_PI)/ticksPerRevolution; // convert to rad/s
            break;
        case tick_s:
            // convert to tick/s
            break;
        case mm_s:
            // convert to mm/s
            break;
        default:
            return (MAXFLOAT);
    }
}

void BrushlessEirbot::setPI(float Kp, float wi, std::chrono::microseconds Te_chrono) {
    TeUsController = Te_chrono;
    float Te = (float) (TeUsController.count() / 1e-6);
    controller = new PIController(Kp, wi, Te);
}

void BrushlessEirbot::setPID(float Kp, float wi, float wb, float wh, std::chrono::microseconds Te_chrono) {
    TeUsController = Te_chrono;
    float Te = (float) (TeUsController.count() / 1e-6);
    controller = new PIDController(Kp, wi, wb, wh, Te);
}

void BrushlessEirbot::displayPinOut() {
    if (_debug){
        std::string buffer("PMW on half-bridges :\n");
        buffer.append(to_string(_pinPWM_AH) + "\t");
        buffer.append(to_string(_pinPWM_AL) + "\n");
        buffer.append(to_string(_pinPWM_BH) + "\t");
        buffer.append(to_string(_pinPWM_BL) + "\n");
        buffer.append(to_string(_pinPWM_CH) + "\t");
        buffer.append(to_string(_pinPWM_CL) + "\n");
        buffer.append("Hall sensors :\n");
        buffer.append(to_string(_pinHall_1) + "\n");
        buffer.append(to_string(_pinHall_2) + "\n");
        buffer.append(to_string(_pinHall_3) + "\n");
        buffer.append("Currents sensors :\n");
        buffer.append(to_string(_pinCurrent_A) + "\n");
        buffer.append(to_string(_pinCurrent_B) + "\n");
        buffer.append(to_string(_pinCurrent_C) + "\n");

        _serial->write(buffer.c_str(), buffer.length());
    }
}

/* *******************************************************************************
 *                              Méthodes primitives
 * *******************************************************************************/

void BrushlessEirbot::decodeHall() {

    hall.h123= (hall.h1<<2) | (hall.h2<<1) | hall.h3;     // for motor sense reading
    if (PWM.sens == clockwise) {                                   // clockwise
        PWM.aH = (hall.h1)  && (!hall.h2);
        PWM.aL = (!hall.h1) && (hall.h2);
        PWM.bH = (hall.h2)  && (!hall.h3);
        PWM.bL = (!hall.h2) && (hall.h3);
        PWM.cH = (!hall.h1) && (hall.h3);
        PWM.cL = (hall.h1)  && (!hall.h3);
    } else if (PWM.sens == antiClockwise) {                        // anti-clockwise
        PWM.aH = (!hall.h1) && (hall.h2);
        PWM.aL = (hall.h1)  && (!hall.h2);
        PWM.bH = (!hall.h2) && (hall.h3);
        PWM.bL = (hall.h2)  && (!hall.h3);
        PWM.cH = (hall.h1)  && (!hall.h3);
        PWM.cL = (!hall.h1) && (hall.h3);
    }

    // Count ticks
    if (hall.h123 != hall.prev_h123) {                            //for sense sign reading
        // 101 -> 100 -> 110 -> 010 -> 011 -> 001 clockwise
        switch (hall.h123)
        {
            case 0b001:
                if (hall.prev_h123 == 0b011) (hall.ticks)++;      // clockwise
                else (hall.ticks)--;
                break;                                            // anti clockwise
            case 0b010:
                if (hall.prev_h123 == 0b110) (hall.ticks)++;
                else (hall.ticks)--;
                break;
            case 0b011:
                if (hall.prev_h123 == 0b010) (hall.ticks)++;
                else (hall.ticks)--;
                break;
            case 0b100:
                if (hall.prev_h123 == 0b101) (hall.ticks)++;
                else (hall.ticks)--;
                break;
            case 0b101:
                if (hall.prev_h123 == 0b001) (hall.ticks)++;
                else (hall.ticks)--;
                break;
            case 0b110:
                if (hall.prev_h123 == 0b100) (hall.ticks)++;
                else (hall.ticks)--;
                break;
            default:
                break;
        }
    }

    hall.prev_h123 = hall.h123;
}

void BrushlessEirbot::updateOutput() {
    if (PWM.aH) {                       //CC1E=1
        timerPWM->CCER |= TIM_CCER_CC1E;
    } else {                            //CC1E=0
        timerPWM->CCER &= ~(TIM_CCER_CC1E);
    }

    if (PWM.aL) {                       //CC1NE=1
        timerPWM->CCER |= TIM_CCER_CC1NE;
    } else {                            //CC1NE=0
        timerPWM->CCER &= ~(TIM_CCER_CC1NE);
    }

    if (PWM.bH) {
        timerPWM->CCER |= TIM_CCER_CC2E;
    } else {
        timerPWM->CCER &= ~(TIM_CCER_CC2E);
    }

    if (PWM.bL) {
        timerPWM->CCER |= TIM_CCER_CC2NE;
    } else {
        timerPWM->CCER &= ~(TIM_CCER_CC2NE);
    }

    if (PWM.cH) {
        timerPWM->CCER |= TIM_CCER_CC3E;
    } else {
        timerPWM->CCER &= ~(TIM_CCER_CC3E);
    }

    if (PWM.cL) {
        timerPWM->CCER |= TIM_CCER_CC3NE;
    } else {
        timerPWM->CCER &= ~(TIM_CCER_CC3NE);
    }
}

/* *******************************************************************************
 *                              Méthodes asservissements
 * *******************************************************************************/

//void BrushlessEirbot::setDutyCycle() {
//    /*
//     * Impose le rapport cyclique pour régler l'intensité magnétique d'accroche du rotor.
//     */
//    if ((data.cmdsat >= 0) && (data.cmdsat <= DC_MAX))          // clockwise
//    {
//        PWM.sens = clockwise;
//        timerPWM->CCR1 = 25;
//        timerPWM->CCR2 = 35;
//        timerPWM->CCR3 = 45;
//    } else if ((data.cmdsat >= DC_MIN) && (data.cmdsat < 0))    // anti clockwise
//    {
//        PWM.sens = antiClockwise;
//        timerPWM->CCR1 = -data.cmdsat;
//        timerPWM->CCR2 = -data.cmdsat;
//        timerPWM->CCR3 = -data.cmdsat;
//    } else                                                      //error state -. PWM off
//    {
//        PWM.sens = clockwise;
//        timerPWM->CCR1 = 0;
//        timerPWM->CCR2 = 0;
//        timerPWM->CCR3 = 0;
//    }
//}

int16_t BrushlessEirbot::calculateSpeed() {
    /*
     * Calculate motor speed using hall effect sensor
     * called every 10ms (100Hz)
     * return measurement_L.speed in tick/s
     * resolution : 1/p mechanical turn  -> 48ticks for 1turn
     */
    if (hall.ticks > 240)
        return (int16_t) MAX_SPEED;
    else if (hall.ticks < -240)
        return (int16_t) MIN_SPEED;
    else
        return 100 * hall.ticks;
}

/* *******************************************************************************
 *                     Cadencement des méthodes primaires et asservissment
 * *******************************************************************************/
void BrushlessEirbot::hallInterrupt() {
    // Lecture Hall sensors
    hall.h1 = HALL_1->read();
    hall.h2 = HALL_2->read();
    hall.h3 = HALL_3->read();

    // Affectation de la séquence d'après
    hall.h123= (hall.h1<<2) | (hall.h2<<1) | hall.h3; // for motor sense reading
    if (PWM.sens == clockwise) {
        PWM.aH = (hall.h1)  && (!hall.h2);
        PWM.aL = (!hall.h1) && (hall.h2);
        PWM.bH = (hall.h2)  && (!hall.h3);
        PWM.bL = (!hall.h2) && (hall.h3);
        PWM.cH = (!hall.h1) && (hall.h3);
        PWM.cL = (hall.h1)  && (!hall.h3);
    } else if (PWM.sens == antiClockwise) {
        PWM.aH = (!hall.h1) && (hall.h2);
        PWM.aL = (hall.h1)  && (!hall.h2);
        PWM.bH = (!hall.h2) && (hall.h3);
        PWM.bL = (hall.h2)  && (!hall.h3);
        PWM.cH = (hall.h1)  && (!hall.h3);
        PWM.cL = (!hall.h1) && (hall.h3);
    }
}

void BrushlessEirbot::_routineController(){

    // TODO Asservissement routine
}



