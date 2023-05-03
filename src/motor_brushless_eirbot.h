/*
 * EIRBOT 2023
 * Mbed Software for controlling Eirbot Brushless
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#ifndef CATIE_SIXTRON_MOTOR_BRUSHLESS_EIRBOT_H
#define CATIE_SIXTRON_MOTOR_BRUSHLESS_EIRBOT_H

#include "mbed.h"
#include "motor/motor_DC.h"
//#include "motor_sensor_hall.h"

namespace sixtron {

    enum position {
        left, right
    };
    enum rotationSens_t {
        clockwise, antiClockwise
    };
    typedef struct {
        bool aH;
        bool aL;
        bool bH;
        bool bL;
        bool cH;
        bool cL;
    } halfBridge_t;
    const halfBridge_t clockwiseSequence[6] = { //  101 -> 001 -> 011 -> 010 -> 110 -> 100
            {false, true,  false, false, true,  false}, // 0b001
            {true,  false, false, true,  false, false}, // 0b010
            {true,  false, false, false, false, true},  // 0b011
            {false, true,  false, false, true,  false}, // 0b100
            {false, true,  true,  false, false, false}, // 0b101
            {false, false, false, true,  true,  false}, // 0b110
    };
    const halfBridge_t antiClockwiseSequence[6] = {
            {false, false, false, true,  true,  false}, // 0b001
            {false, true,  true,  false, false, false}, // 0b010
            {false, true,  false, false, true,  false}, // 0b011
            {true,  false, false, false, false, true},  // 0b100
            {true,  false, false, true,  false, false}, // 0b101
            {false, false, true,  false, false, true},  // 0b110
    };

#define DEFAULT_MOTOR_MAX_PWM 1.0f // max PWM with mbed is 1.0f

    class MotorBrushlessEirbot: public MotorDC {

    public:
		MotorBrushlessEirbot(float rate_dt,
							 position motor_position,
							 PID_params motor_pid,
							 float max_pwm = DEFAULT_MOTOR_MAX_PWM):
							 MotorDC(rate_dt, motor_pid, max_pwm),
							 _positionMotor(motor_position), _updateRate_dt(rate_dt) {};

		void setSpeed(float speed_ms) override;

    public:
        void initHardware() override;
        void setPWM(float pwm) override;
        float getSensorSpeed() override;
		int32_t getHALLticks();

    private:
		inline float ticks2Meters(float ticks) const
		{
			return (ticks * (1.0f / _tickPerMeters));
		}
		float _wheelPerimeter, _tickPerMeters;
		float _motorResolution, _motorWheelRadius;
		float _updateRate_dt = 0.0f;
//		int64_t _sensorResolution = 0;
//		int64_t _sensorDirection = 0;

		int64_t _sensorTickCount = 0;
		int64_t _sensorTickCountOld = 0;

		TIM_TypeDef *_tim;
        position _positionMotor;
        rotationSens_t _sens;
        InterruptIn *HALL_1;
        InterruptIn *HALL_2;
        InterruptIn *HALL_3;
        void hallInterrupt();
        void halfBridgeApply(halfBridge_t halfBridgeConfig);
        const halfBridge_t halfBridgeZEROS = {0, 0, 0, 0, 0, 0};
        volatile uint8_t _hallWord_previous;
        volatile int32_t _ticks;
        PinName _pinHall_1;
        PinName _pinHall_2;
        PinName _pinHall_3;
		bool force_hall_update;
    };

}; // namespace sixtron
#endif // CATIE_SIXTRON_MOTOR_BRUSHLESS_EIRBOT_H