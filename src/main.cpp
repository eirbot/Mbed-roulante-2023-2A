// ============= INCLUDES =================
#include "mbed.h"
//#include "BrushlessEirbot.hpp"
//#include "SerialPlot.hpp"
#include "motor_brushless_eirbot.h"

// ============= DEFINITIONS =================
DigitalOut led(LED1);

//BufferedSerial pc(USBTX, USBRX, 115200);
//EventFlags flagDebug;
//SerialPlot debugPlot(&pc, &flagDebug);
//BrushlessEirbot motorL(&debugPlot, Left, 78);

// Motor MBED and HALL sensorconfig
#define MOTOR_UPDATE_RATE 10ms // 20ms for 50Hz
#define MOTOR_FLAG 0x01
#define ENC_HALL_RESOLUTION 65536 // max uint16
#define MOTOR_RESOLUTION 48
#define MOTOR_REDUCTION 14
#define WHEEL_RESOLUTION float(MOTOR_RESOLUTION*MOTOR_REDUCTION)
#define WHEEL_RADIUS (0.078f/2.0f)
#define MAX_PWM 0.7f
Ticker MotorUpdateTicker;
EventFlags MotorFlag;
Thread motorThread(osPriorityRealtime);
sixtron::MotorBrushlessEirbot *motor_left;
sixtron::MotorBrushlessEirbot *motor_right;
bool motor_init_done = false;

void MotorFlagUpdate() {
    MotorFlag.set(MOTOR_FLAG);
}

void motorThreadMain() {
    // First, convert the rate of the loop in seconds [float]
    auto f_secs = std::chrono::duration_cast<std::chrono::duration<float>>(MOTOR_UPDATE_RATE);
    float dt_pid = f_secs.count();

    // Create a motor object
    sixtron::PID_params pid_motor_params;
    pid_motor_params.Kp = 6.0f;
    pid_motor_params.Ki = 10.0f;
    pid_motor_params.Kd = 0.0f;
    pid_motor_params.Kf = 2.0f;
    pid_motor_params.dt_seconds = dt_pid;
    pid_motor_params.ramp = 1.0f * dt_pid;

    motor_left = new sixtron::MotorBrushlessEirbot(
            dt_pid,
			sixtron::position::left,
			pid_motor_params,
            ENC_HALL_RESOLUTION,
            WHEEL_RESOLUTION,
            WHEEL_RADIUS,
            DIR_NORMAL,
            MAX_PWM);

    motor_right = new sixtron::MotorBrushlessEirbot(
            dt_pid,
            sixtron::position::right,
            pid_motor_params,
            ENC_HALL_RESOLUTION,
            WHEEL_RESOLUTION,
            WHEEL_RADIUS,
            DIR_INVERTED,
            MAX_PWM);

    // Init motor and sensor
    motor_left->init();
    motor_right->init();
    ThisThread::sleep_for(500ms);
    motor_init_done = true;

    while (true) {
        // wait for the flag trigger
        MotorFlag.wait_any(MOTOR_FLAG);

        // Update sensor motor
        motor_left->update();
        motor_right->update();
    }
}

// Just for the debug
void set_motor_target(float speed_ms) {
//    printf("Applying %2.3f m/s to the motor.\n", speed_ms);
    motor_left->setSpeed(speed_ms);
    motor_right->setSpeed(speed_ms);
}

void set_motor_target(float speed_left, float speed_right) {
//    printf("Applying %2.3f m/s to the motor.\n", speed_ms);
    motor_left->setSpeed(speed_left);
    motor_right->setSpeed(speed_right);
}

int main() {
    // Start the thread for motor control
    motorThread.start(motorThreadMain);

    // Setup ticker to update the motor base flag at exactly the defined rate
    MotorUpdateTicker.attach(&MotorFlagUpdate, MOTOR_UPDATE_RATE);

    printf("Waiting for the motor to be setup ...\n");
    while (!motor_init_done);
    printf("Motor init done, continue with setting targets.\n");

    while (true) {

        set_motor_target(+0.2f);
        ThisThread::sleep_for(3s);

        set_motor_target(-0.1f, +0.1f);
        ThisThread::sleep_for(3s);


//        set_motor_target(-1.0f);
//        ThisThread::sleep_for(4s);
//
//        set_motor_target(0.0f);
//        ThisThread::sleep_for(2s);
//
//        set_motor_target(-0.4f, +0.4f);
//        ThisThread::sleep_for(2s);
//
//        set_motor_target(-0.6f, +0.6f);
//        ThisThread::sleep_for(3s);
//
//        set_motor_target(-1.0f, +1.0f);
//        ThisThread::sleep_for(4s);
//
//        set_motor_target(0.0f);
//        ThisThread::sleep_for(2s);
    }
}
