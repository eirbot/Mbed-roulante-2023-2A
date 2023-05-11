/*
 * EIRBOT 2023
 * Mbed Software for controlling Eirbot Brushless
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#include "odometry_eirbot.h"

namespace sixtron {

    void OdometryEirbot::init() {

        // nothing to init, already done by motors
    }

    void OdometryEirbot::update() {

        // Update Halls sensors
        _motor_base->updateHalls();

        // right is inverted !!
        compute(_motor_base->getSensorObj(MOTOR_LEFT)->getTickCount(),
                -_motor_base->getSensorObj(MOTOR_RIGHT)->getTickCount());
    }
}
