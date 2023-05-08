// ============= INCLUDES =================
#include "mbed.h"
#include "odometry_eirbot.h"
#include "motor_base_eirbot.h"

// ============= DEFINITIONS =================
DigitalOut led(LED1);

// Motor MBED and HALL sensorconfig
#define MOTOR_UPDATE_RATE 10ms // 10ms for 100Hz
#define MOTOR_FLAG 0x01
Ticker MotorUpdateTicker;
EventFlags MotorFlag;
Thread motorThread(osPriorityRealtime);
sixtron::MotorBaseEirbot *base_eirbot;
bool motor_init_done = false;

// Odometry
sixtron::OdometryEirbot *odom;

void MotorFlagUpdate() {
    MotorFlag.set(MOTOR_FLAG);
}

void motorThreadMain() {
    // First, convert the rate of the loop in seconds [float]
    auto f_secs = std::chrono::duration_cast<std::chrono::duration<float>>(MOTOR_UPDATE_RATE);
    float dt_pid = f_secs.count();
    float rate_hz = dt_pid / 1.0f;

    // Create Eirbot Base. this will create and init both brushless motors + hall sensors.
    base_eirbot = new sixtron::MotorBaseEirbot(dt_pid);
    base_eirbot->init();

    // Init odometry
    odom = new sixtron::OdometryEirbot(rate_hz,
                                       base_eirbot->getSensorObj(MOTOR_LEFT),
                                       base_eirbot->getSensorObj(MOTOR_RIGHT),
                                       WHEEL_RESOLUTION,
                                       WHEEL_RADIUS,
                                       WHEELS_DISTANCE);
    odom->init();

    ThisThread::sleep_for(200ms);
    motor_init_done = true;

    int printf_debug_incr = 0;
    while (true) {
        // wait for the flag trigger
        MotorFlag.wait_any(MOTOR_FLAG);

        // Update hall sensors
        base_eirbot->updateHalls();

        // Update Odom
        odom->update();

        // Update motors
        base_eirbot->update();

        // debug
        printf_debug_incr++;
        if(printf_debug_incr > 5){
            printf_debug_incr = 0;
            printf("Odom: x=%3.2fm, y=%3.2fm, theta=%3.2frad\n",
                   odom->getX(),
                   odom->getY(),
                   odom->getTheta());
        }
    }
}

int main() {
    // Start the thread for motor control
    motorThread.start(motorThreadMain);

    // Setup ticker to update the motor base flag at exactly the defined rate
    MotorUpdateTicker.attach(&MotorFlagUpdate, MOTOR_UPDATE_RATE);

    printf("Waiting for the motor to be setup ...\n");
    while (!motor_init_done);
    printf("Motor init done, continue with setting targets.\n");
    ThisThread::sleep_for(500ms);

    sixtron::target_speeds target;
    target.cmd_lin = 0.0f;
    target.cmd_rot = 0.0f;

    while (true) {

        target.cmd_lin = 0.2f;
        target.cmd_rot = 0.0f;
        base_eirbot->setTargetSpeeds(target);
        ThisThread::sleep_for(2s);

        target.cmd_lin = 0.0f;
        target.cmd_rot = 0.785f;
        base_eirbot->setTargetSpeeds(target);
        ThisThread::sleep_for(2s);

    }
}
