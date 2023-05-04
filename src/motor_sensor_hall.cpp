/*
 * EIRBOT 2023
 * Mbed Software for controlling Eirbot Brushless
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#include "motor_sensor_hall.h"

namespace sixtron {

    void MotorSensorHall::initSensor() {
        // Nothing to init ...
    }

    uint16_t MotorSensorHall::getSensorValue() {
        return *_motor_hall_ticks;
    }

} // namespace sixtron
