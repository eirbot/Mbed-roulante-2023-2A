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

namespace sixtron {

#define DEFAULT_MOTOR_MAX_PWM 1.0f // max PWM with mbed is 1.0f

    class MotorBrushlessEirbot: public MotorDC {

    public:
		MotorBrushlessEirbot(float rate_dt,
                PID_params motor_pid,
                float max_pwm = DEFAULT_MOTOR_MAX_PWM):
                MotorDC(rate_dt, motor_pid, max_pwm) {};

    private:
        void initHardware() override;
        void setPWM(float pwm) override;
        float getSensorSpeed() override;
		int32_t getHALLticks();

    private:

    };

} // namespace sixtron

#endif // CATIE_SIXTRON_MOTOR_BRUSHLESS_EIRBOT_H
