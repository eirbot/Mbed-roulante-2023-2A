/*
 * EIRBOT 2023
 * Mbed Software for controlling Eirbot Brushless
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#ifndef CATIE_SIXTRON_MOTORSENSOR_MBED_HALL_H
#define CATIE_SIXTRON_MOTORSENSOR_MBED_HALL_H

#include "mbed.h"
#include "motor_sensor_encoder.h"

namespace sixtron {

#define DIR_NORMAL (+1)
#define DIR_INVERTED (-1)

    class MotorSensorHall : public MotorSensorEncoder {
    public:
        MotorSensorHall(float rate_dt,
                        uint8_t *hall_ticks,
                        int32_t sensorResolution,
                        float motorResolution,
                        float motorWheelRadius,
                        int encDirection = DIR_NORMAL) :
                MotorSensorEncoder(rate_dt, sensorResolution, motorResolution,
                                   motorWheelRadius, encDirection),
                _motor_hall_ticks(hall_ticks) {};

    protected:
        void initSensor() override;

        uint16_t getSensorValue() override;

    private:

        uint8_t *_motor_hall_ticks;

    };

} // namespace sixtron

#endif // CATIE_SIXTRON_MOTORSENSOR_MBED_HALL_H
