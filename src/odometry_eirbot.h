/*
 * EIRBOT 2023
 * Mbed Software for controlling Eirbot Brushless
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#ifndef ODOMETRY_EIRBOT_H
#define ODOMETRY_EIRBOT_H

#include "mbed.h"
#include "odometry/odometry_two_encoders.h"
#include "motor_brushless_eirbot.h"
#include "motor_base_eirbot.h"

namespace sixtron {

    class OdometryEirbot : public OdometryTwoEncoders {

    public:
        OdometryEirbot(float rate_hz,
                       sixtron::MotorBaseEirbot *base,
                       float wheel_resolution,
                       float motor_wheel_radius,
                       float wheels_distance) :
                OdometryTwoEncoders(rate_hz,
                                    wheel_resolution,
                                    motor_wheel_radius,
                                    wheels_distance),
                _motor_base(base) {};

        ~OdometryEirbot() = default;

        // Specific for Eirbot, HALL already init by motors
        void init() override;

        // Specific for Eirbot , get HALL tick count
        void update() override;

    private:
        MotorBaseEirbot *_motor_base;
    };
}

#endif // ODOMETRY_EIRBOT_H
