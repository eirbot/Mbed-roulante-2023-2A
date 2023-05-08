/*
 * EIRBOT 2023
 * Mbed Software for controlling Eirbot Brushless
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#include "motor_base_eirbot.h"

namespace sixtron {

    void MotorBaseEirbot::init() {

        // Create a motor object
        sixtron::PID_params pid_motor_params;
        pid_motor_params.Kp = 6.0f;
        pid_motor_params.Ki = 10.0f;
        pid_motor_params.Kd = 0.0f;
        pid_motor_params.Kf = 10.0f;
        pid_motor_params.dt_seconds = _dt_pid;
        pid_motor_params.ramp = 1.0f * _dt_pid;

        _motor_left = new sixtron::MotorBrushlessEirbot(
                _dt_pid,
                sixtron::motor_position::left,
                pid_motor_params,
                ENC_HALL_RESOLUTION,
                WHEEL_RESOLUTION,
                WHEEL_RADIUS,
                DIR_NORMAL,
                MAX_PWM);

        _motor_right = new sixtron::MotorBrushlessEirbot(
                _dt_pid,
                sixtron::motor_position::right,
                pid_motor_params,
                ENC_HALL_RESOLUTION,
                WHEEL_RESOLUTION,
                WHEEL_RADIUS,
                DIR_INVERTED,
                MAX_PWM);

        // Init motor and sensor
        _motor_left->init();
        _motor_right->init();

        ThisThread::sleep_for(200ms);

        _motor_left->setSpeed(0.0f);
        _motor_right->setSpeed(0.0f);

        _motor_left->start();
        _motor_right->start();
    }

    void MotorBaseEirbot::update() {

        computeMotorSpeeds();

        _motor_left->setSpeed(_targetSpeedMotorLeft);
        _motor_right->setSpeed(_targetSpeedMotorRight);

        _motor_left->update();
        _motor_right->update();
    }
    void MotorBaseEirbot::updateHalls() {
        // Update hall sensor counter.
        // DO NOT UPDATE FROM getSensorObj().update() !
        _motor_left->updateHallSensor();
        _motor_right->updateHallSensor();
    }
    MotorSensorHall *MotorBaseEirbot::getSensorObj(int motor) {

        if (motor == MOTOR_LEFT) {
            return _motor_left->getSensorObj();
        } else {
            return _motor_right->getSensorObj();
        }
    }
}
