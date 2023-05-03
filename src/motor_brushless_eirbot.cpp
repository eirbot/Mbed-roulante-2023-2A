/*
 * EIRBOT 2023
 * Mbed Software for controlling Eirbot Brushless
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#include "motor_brushless_eirbot.h"

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




namespace sixtron {

    void MotorBrushlessEirbot::initHardware() {
		force_hall_update = true;

        // Init Motor PWM
        if (_positionMotor == left) {
            // Configuration des Hall Sensors
            _pinHall_1 = PB_2;
            _pinHall_2 = PB_5;
            _pinHall_3 = PB_6;
            _tim = TIM1;
            TIMER1_init();
        }
        else if (_positionMotor == right) {
            _pinHall_1 = PB_7;
            _pinHall_2 = PB_8;
            _pinHall_3 = PB_9;
            _tim = TIM8;
            TIMER8_init();
        }

        _hallWord_previous = 0b000;
        halfBridgeApply(halfBridgeZEROS);

        HALL_1 = new InterruptIn(_pinHall_1, PullNone);
        HALL_2 = new InterruptIn(_pinHall_2, PullNone);
        HALL_3 = new InterruptIn(_pinHall_3, PullNone);
        HALL_1->rise(callback(this, &MotorBrushlessEirbot::hallInterrupt));
        HALL_2->rise(callback(this, &MotorBrushlessEirbot::hallInterrupt));
        HALL_3->rise(callback(this, &MotorBrushlessEirbot::hallInterrupt));
    }

    float MotorBrushlessEirbot::getSensorSpeed() {
        // réducteur 14
        // nb tick par tour non réduit 48
        return 14*48;
    }

	void MotorBrushlessEirbot::setSpeed(float speed_ms)
	{
		_targetSpeed = speed_ms;
		force_hall_update = true;
	}


	void MotorBrushlessEirbot::setPWM(float pwm) {
        // update hardware motor PWM

        if (pwm >= 0) { _sens = clockwise; }
        else { _sens = antiClockwise; }

        // Conversion vers du uint8_t
        uint8_t dutyCycle_int = (uint8_t) (abs(pwm) * 100);

        // Saturation
        if (dutyCycle_int >= 90) { dutyCycle_int = 90; }

        _tim->CCR1 = dutyCycle_int;
        _tim->CCR2 = dutyCycle_int;
        _tim->CCR3 = dutyCycle_int;

		if(force_hall_update){
			hallInterrupt();
			force_hall_update = false;
		}

    }

    int32_t MotorBrushlessEirbot::getHALLticks() {
        return _ticks;
	}

    void MotorBrushlessEirbot::hallInterrupt() {
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
} // namespace sixtron
