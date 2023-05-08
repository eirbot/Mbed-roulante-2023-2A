/*
 * EIRBOT 2023
 * Mbed Software for controlling Eirbot Brushless
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#ifndef MOTOR_BASE_EIRBOT_H
#define MOTOR_BASE_EIRBOT_H

#include "mbed.h"
#include "motor_brushless_eirbot.h"
#include "motor_base/motor_base_two_wheels.h"

namespace sixtron {

#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

#define ENC_HALL_RESOLUTION 256 // max uint8
#define MOTOR_RESOLUTION 48
#define MOTOR_REDUCTION 14
#define WHEEL_RESOLUTION float(MOTOR_RESOLUTION*MOTOR_REDUCTION)
#define WHEEL_RADIUS (0.080f/2.0f)
#define MAX_PWM 0.7f // With MBED, pwm command between -1.0f and +1.0f max !
#define WHEELS_DISTANCE 0.30f

    class MotorBaseEirbot: public MotorBaseTwoWheels {

    public:
        MotorBaseEirbot(
                float rate_dt):
                MotorBaseTwoWheels(WHEELS_DISTANCE),
                _dt_pid(rate_dt) {};

        ~MotorBaseEirbot() = default;

        // Specific for Eirbot, uses two PIDs to control each motor.
        void init() override;

        // Specific for Eirbot.
        void update() override;

        // Specific for Eirbot, because hall are implemented inside motor class.
        void updateHalls();

        // Odometry need the total tick count for both motor to work.
        MotorSensorHall *getSensorObj(int motor);

    private:
        float _dt_pid; // in [s]

        MotorBrushlessEirbot *_motor_left;
        MotorBrushlessEirbot *_motor_right;

    };
}

#endif // MOTOR_BASE_EIRBOT_H
