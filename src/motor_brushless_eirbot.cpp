/*
 * EIRBOT 2023
 * Mbed Software for controlling Eirbot Brushless
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================


#include "motor_brushless_eirbot.h"
#include "tim.h"

#ifdef  USE_CUSTOM_HAL_INTERRUPT
static sixtron::MotorBrushlessEirbot *motor_left_obj;
static sixtron::MotorBrushlessEirbot *motor_right_obj;
#endif


void TIMER1_init() {
    MX_TIM1_Init();

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

#ifdef  USE_CUSTOM_HAL_INTERRUPT
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure HALL inputs pins */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif
}

void TIMER8_init() {
    MX_TIM8_Init();

    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);

#ifdef  USE_CUSTOM_HAL_INTERRUPT
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure HALL inputs pins */
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif
}

#ifdef  USE_CUSTOM_HAL_INTERRUPT

void custom_EXTI_IRQHandler(void) {

    //PB2 -> Motor Left
    if (LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_2)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
        motor_left_obj->hallInterrupt();
    }
    //PB5 -> Motor Left
    if (LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_5)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_5);
        motor_left_obj->hallInterrupt();
    }
    //PB6 -> Motor Left
    if (LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_6)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_6);
        motor_left_obj->hallInterrupt();
    }

    // PB7 -> Motor Right
    if (LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_7)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_7);
        motor_right_obj->hallInterrupt();
    }
    // PB8 -> Motor Right
    if (LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_8)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_8);
        motor_right_obj->hallInterrupt();
    }
    // PB9 -> Motor Right
    if (LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_9)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_9);
        motor_right_obj->hallInterrupt();
    }
}

#endif

namespace sixtron {

    void MotorBrushlessEirbot::initHardware() {
        force_hall_update = true;

        _sensor_hall.init();

        // Init Motor PWM
        if (_positionMotor == left) {
            // Configuration des Hall Sensors

#ifdef  USE_CUSTOM_HAL_INTERRUPT
            gpio_hall_u = GPIO_PIN_2;
            gpio_hall_v = GPIO_PIN_5;
            gpio_hall_w = GPIO_PIN_6;

            /* EXTI interrupt init*/
            HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
            NVIC_SetVector(EXTI2_IRQn, (uint32_t) &custom_EXTI_IRQHandler);
            HAL_NVIC_EnableIRQ(EXTI2_IRQn);

            HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
            NVIC_SetVector(EXTI9_5_IRQn, (uint32_t) &custom_EXTI_IRQHandler);
            HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

            motor_left_obj = this;
#else
            _pinHall_1 = PB_2;
            _pinHall_2 = PB_5;
            _pinHall_3 = PB_6;
#endif
            _tim = TIM1;
            TIMER1_init();

        } else if (_positionMotor == right) {


#ifdef  USE_CUSTOM_HAL_INTERRUPT
            gpio_hall_u = GPIO_PIN_7;
            gpio_hall_v = GPIO_PIN_8;
            gpio_hall_w = GPIO_PIN_9;

            motor_right_obj = this;
#else
            _pinHall_1 = PB_7;
            _pinHall_2 = PB_8;
            _pinHall_3 = PB_9;
#endif
            _tim = TIM8;
            TIMER8_init();

        }

        halfBridgeApply(halfBridgeZEROS);

#ifndef  USE_CUSTOM_HAL_INTERRUPT
        HALL_1 = new InterruptIn(_pinHall_1, PullNone);
        HALL_2 = new InterruptIn(_pinHall_2, PullNone);
        HALL_3 = new InterruptIn(_pinHall_3, PullNone);
        HALL_1->rise(callback(this, &MotorBrushlessEirbot::hallInterrupt));
        HALL_2->rise(callback(this, &MotorBrushlessEirbot::hallInterrupt));
        HALL_3->rise(callback(this, &MotorBrushlessEirbot::hallInterrupt));
        HALL_1->fall(callback(this, &MotorBrushlessEirbot::hallInterrupt));
        HALL_2->fall(callback(this, &MotorBrushlessEirbot::hallInterrupt));
        HALL_3->fall(callback(this, &MotorBrushlessEirbot::hallInterrupt));
#endif
    }

    void MotorBrushlessEirbot::updateHallSensor() {
        _hall_ticks_fixed = _hall_ticks; // need to fix the value when doing calculs, because of interrupts
        _sensor_hall.update();
    }

    float MotorBrushlessEirbot::getSensorSpeed() {
        return _sensor_hall.getSpeed();
    }

    void MotorBrushlessEirbot::setSpeed(float speed_ms) {
        if (_motorDir == DIR_NORMAL) {
            _targetSpeed = speed_ms;
        } else {
            _targetSpeed = -speed_ms;
        }
        if (speed_ms != _last_speed) {
            force_hall_update = true;
        }
        _last_speed = speed_ms;
    }

    void MotorBrushlessEirbot::setPWM(float pwm) {
        // update hardware motor PWM

        if (force_hall_update &&
            ((pwm >= FORCE_HALL_MIN_PWM) || (pwm <= FORCE_HALL_MIN_PWM))) {
            hallInterrupt();
            force_hall_update = false;
        }

        if (pwm > 0.01f) {
            _sens = clockwise;
        } else if (pwm < -0.01f) {
            _sens = antiClockwise;
        } else {
            _sens = none;
        }

        // Conversion vers uint8_t
        uint8_t dutyCycle_int = (uint8_t) (abs(pwm) * 100);

        // Saturation
        if (dutyCycle_int >= 90) { dutyCycle_int = 90; }

        _tim->CCR1 = dutyCycle_int;
        _tim->CCR2 = dutyCycle_int;
        _tim->CCR3 = dutyCycle_int;

    }

    static inline int getSector(uint8_t hallWord) {
        static const unsigned int hall_to_phase[6] = {0, 2, 1, 4, 5, 3};

        if ((hallWord >= 1) && (hallWord <= 6)) {
            return hall_to_phase[hallWord - 1];
        } else {
            return -1;
        }
    }

    void MotorBrushlessEirbot::updateTicks(uint8_t hallWord) {
        int delta, sector;

        sector = getSector(hallWord);

        if (sector == -1) {
//            _hall_ticks = 42;
            return; // ne devrait jamais arriver ... sinon problème de câbles HALL
        }

        delta = sector - _old_sector;
        _old_sector = sector;

        // delta range check
        if (delta <= -3) {
            _hall_ticks += delta + 6;
        } else if (delta >= 3) {
            _hall_ticks += delta - 6;
        } else {
            _hall_ticks += delta;
        }
    }

    void MotorBrushlessEirbot::hallInterrupt() {
        // Lecture Hall sensors

#ifdef  USE_CUSTOM_HAL_INTERRUPT
        uint8_t hallWord = ((((GPIOB->IDR & gpio_hall_u) != 0u) ? 1 : 0) << 2) |
                           ((((GPIOB->IDR & gpio_hall_v) != 0u) ? 1 : 0) << 1) |
                           ((((GPIOB->IDR & gpio_hall_w) != 0u) ? 1 : 0));
#else
        uint8_t hallWord = (((HALL_1->read() != 0u) ? 1 : 0) << 2) |
                           (((HALL_2->read() != 0u) ? 1 : 0) << 1) |
                           (((HALL_3->read() != 0u) ? 1 : 0));
#endif

        // Mis à jours des ticks en fonction du secteur des halls
        updateTicks(hallWord);

        // Affectation de la séquence d'après
        halfBridge_t halfBridge;
        if (_sens == clockwise) {
            halfBridge = clockwiseSequence[hallWord - 1];
        } else if (_sens == antiClockwise) {
            halfBridge = antiClockwiseSequence[hallWord - 1];
        } else {
            halfBridge = halfBridgeZEROS;
            force_hall_update = true; // because motor pas bougé donc hall pas se mettre à jour tout seul
        }

        halfBridgeApply(halfBridge);
    }

    void MotorBrushlessEirbot::halfBridgeApply(halfBridge_t halfBridgeConfig) {
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

    MotorSensorHall *MotorBrushlessEirbot::getSensorObj() {
        return &_sensor_hall;
    }
} // namespace sixtron
