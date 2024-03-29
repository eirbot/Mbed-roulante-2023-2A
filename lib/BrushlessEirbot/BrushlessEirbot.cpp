#include "BrushlessEirbot.hpp"
#include "tim.h"

void TIMER1_init() {
    MX_TIM1_Init();

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void TIMER8_init() {
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
    if (!_debug) {
        _SerialPlotDebug = nullptr; // pas de debug
    }

    _positionMotor = position_motor;
    setStateController(desactivated);
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

        _tim = TIM1;
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

        _tim = TIM8;
        TIMER8_init();
    }

    _hallWord_previous = 0b000;
    halfBridgeApply(halfBridgeZEROS);

    HALL_1 = new InterruptIn(_pinHall_1, PullNone);
    HALL_2 = new InterruptIn(_pinHall_2, PullNone);
    HALL_3 = new InterruptIn(_pinHall_3, PullNone);
    HALL_1->rise(callback(this, &BrushlessEirbot::hallInterrupt));
    HALL_2->rise(callback(this, &BrushlessEirbot::hallInterrupt));
    HALL_3->rise(callback(this, &BrushlessEirbot::hallInterrupt));

    Current_A = new AnalogIn(_pinCurrent_A);
    Current_B = new AnalogIn(_pinCurrent_B);
    Current_C = new AnalogIn(_pinCurrent_C);

    _tickerController.attach(callback(this, &BrushlessEirbot::_routineController), TeUsController);

//    setPI(0.018, 7, TeUsController);
    speedRef = 0;
    Te = (float) TeUsController.count() * float(1e-6);
}

BrushlessEirbot::BrushlessEirbot(SerialPlot* debug, position position_motor, float wheelDiameterMm) : BrushlessEirbot(
        position_motor, wheelDiameterMm) {
    _debug = true;
    _SerialPlotDebug = debug;

    _SerialPlotDebug->addVariables(prtSpeed);

}

BrushlessEirbot::~BrushlessEirbot() {
    _tickerController.detach();
    delete HALL_1;
    delete HALL_2;
    delete HALL_3;
    delete Current_A;
    delete Current_B;
    delete Current_C;
    delete _SerialPlotDebug;
    delete _tim;
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
    _sens = clockwise;
    speedRef = consigne;
}

void BrushlessEirbot::setStateController(state stateController) {
    _stateController = stateController;
}

float BrushlessEirbot::getVelocity(unitVelocity unit) {
    // TODO Fixé le timer pour résoudre le problème temporel
    // FIXME mesure en dehors du controller
    uint32_t tickCpy = _ticks;
    _ticks = 0;
    switch (unit) {
        case rad_s:
            if (_sens == clockwise) {
                return (float) (tickCpy * M_2_PI / (TicksPerRevolution))/Te; // convert to rad/s
            } else {
                return (float) (-tickCpy * M_2_PI / (TicksPerRevolution))/Te; // convert to rad/s
            }
            break;

        case tick_s:
            if (_sens == clockwise) {
                return (float) (tickCpy)/Te; // convert to mm/s
            } else {
                return (float) (-tickCpy)/Te; // convert to mm/s
            }
            break;
        case mm_s:
            if (_sens == clockwise) {
                return (float) (tickCpy * M_2_PI * _wheelDiameterMm)/Te; // convert to mm/s
            } else {
                return (float) (-tickCpy * M_2_PI * _wheelDiameterMm)/Te; // convert to mm/s
            }
            break;
    }
}

void BrushlessEirbot::setPI(float Kp, float wi, std::chrono::microseconds Te_chrono) {
    TeUsController = Te_chrono;
    float Te = (float) TeUsController.count() * float(1e-6);
    controller = new PIController(Kp, wi, Te);
}

void BrushlessEirbot::setPID(float Kp, float wi, float wb, float wh, std::chrono::microseconds Te_chrono) {
    TeUsController = Te_chrono;
    float Te = (float) TeUsController.count() * float(1e-6);
    controller = new PIDController(Kp, wi, wb, wh, Te);
}


void BrushlessEirbot::setDutyCycle(float dutyCycle) {
    // Configuration du sens de rotation
    if (dutyCycle >= 0) { _sens = clockwise; }
    else { _sens = antiClockwise; }

    // Conversion vers du uint8_t
    uint8_t dutyCycle_int = (uint8_t) (abs(dutyCycle) * 100); //FIXME Augmenter la résolution du comparateur (penser à changer prescaler)

    // Saturation
    if (dutyCycle_int >= DutyCycleMAX) { dutyCycle_int = DutyCycleMAX; }

    _tim->CCR1 = dutyCycle_int;
    _tim->CCR2 = dutyCycle_int;
    _tim->CCR3 = dutyCycle_int;
}


/* *******************************************************************************
 *                     Cadencement des méthodes primaires et asservissement
 * *******************************************************************************/
void BrushlessEirbot::halfBridgeApply(halfBridge_t halfBridgeConfig) {
    // Mise en œuvre des commandes
    if (halfBridgeConfig.aH) {                //CC1E=1
        _tim->CCER |= TIM_CCER_CC1E;
    } else {                                  //CC1E=0
        _tim->CCER &= ~(TIM_CCER_CC1E);
    }

    if (halfBridgeConfig.aL) {                //CC1NE=1
        _tim->CCER |= TIM_CCER_CC1NE;
    } else {                                  //CC1NE=0
        _tim->CCER &= ~(TIM_CCER_CC1NE);
    }

    if (halfBridgeConfig.bH) {
        _tim->CCER |= TIM_CCER_CC2E;
    } else {
        _tim->CCER &= ~(TIM_CCER_CC2E);
    }

    if (halfBridgeConfig.bL) {
        _tim->CCER |= TIM_CCER_CC2NE;
    } else {
        _tim->CCER &= ~(TIM_CCER_CC2NE);
    }

    if (halfBridgeConfig.cH) {
        _tim->CCER |= TIM_CCER_CC3E;
    } else {
        _tim->CCER &= ~(TIM_CCER_CC3E);
    }

    if (halfBridgeConfig.cL) {
        _tim->CCER |= TIM_CCER_CC3NE;
    } else {
        _tim->CCER &= ~(TIM_CCER_CC3NE);
    }
}

void BrushlessEirbot::hallInterrupt() {
    // Lecture Hall sensors
    volatile uint8_t hallWord = (HALL_1->read() << 2) | (HALL_2->read() << 1) | HALL_3->read();

    // Affectation de la séquence d'après
    halfBridge_t halfBridge = {false, false, false, false, false, false};
    if (_sens == clockwise) {
        halfBridge = clockwiseSequence[hallWord - 1];
        _ticks++; // Indentation des ticks
    } else if (_sens == antiClockwise) {
        halfBridge = antiClockwiseSequence[hallWord - 1];
        _ticks--; // Indentation des ticks
    }

    halfBridgeApply(halfBridge);

    _hallWord_previous = hallWord; // Retenir la valeur précédente
}

void BrushlessEirbot::_routineController() {
    *prtSpeed = getVelocity(mm_s);
    volatile float fconsigne = 0;

    if (_stateController == activated) {
        fconsigne = float(0.001) * (speedRef - *prtSpeed);
        setDutyCycle(fconsigne);
    }
    if (_debug) {
        flagDebug.set(0x0A);
    }
}