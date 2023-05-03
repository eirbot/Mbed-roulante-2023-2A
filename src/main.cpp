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

// Motor MBED config
#define MOTOR_UPDATE_RATE 10ms
#define MOTOR_FLAG 0x01
Ticker MotorUpdateTicker;
EventFlags MotorFlag;
Thread motorThread(osPriorityNormal);
sixtron::MotorBrushlessEirbot* motor_left;
bool motor_init_done = false;

void MotorFlagUpdate()
{
    MotorFlag.set(MOTOR_FLAG);
}

void motorThreadMain()
{
    // First, convert the rate of the loop in seconds [float]
    auto f_secs = std::chrono::duration_cast<std::chrono::duration<float>>(MOTOR_UPDATE_RATE);
    float dt_pid = f_secs.count();

    // Create a motor object
    sixtron::PID_params pid_motor_params;
    pid_motor_params.Kp = 1.0f;
    pid_motor_params.Ki = 0.0f;
    pid_motor_params.Kd = 0.0f;
    pid_motor_params.dt_seconds = dt_pid;
    //pid_motor_params.ramp = 1.0f * dt_pid;

	motor_left = new sixtron::MotorBrushlessEirbot(
			dt_pid, sixtron::position::left, pid_motor_params, 0.6f);

    // Init motor and sensor
    motor_left->init();
    motor_init_done = true;

    while (true) {
        // wait for the flag trigger
        MotorFlag.wait_any(MOTOR_FLAG);

        // Update sensor motor
        motor_left->update();

		printf("speed=%2.4f\n",motor_left->getSpeed());
    }
}

// Just for the debug
void set_motor_target(float speed_ms)
{
    printf("Applying %2.3f m/s to the motor.\n", speed_ms);
    //motor_right->setSpeed(speed_ms);
	motor_left->setPWM(speed_ms);
}

int main()
{
    // Start the thread for motor control
    motorThread.start(motorThreadMain);

    // Setup ticker to update the motor base flag at exactly the defined rate
    MotorUpdateTicker.attach(&MotorFlagUpdate, MOTOR_UPDATE_RATE);

    printf("Waiting for the motor to be setup ...\n");
    while (!motor_init_done);
    printf("Motor init done, continue with setting targets.\n");

    while (true) {

        set_motor_target(0.5f);
        ThisThread::sleep_for(3s);

        set_motor_target(0.0f);
        ThisThread::sleep_for(1s);

        set_motor_target(-0.5f);
        ThisThread::sleep_for(3s);

        set_motor_target(0.0f);
        ThisThread::sleep_for(1s);

//        set_motor_target(-0.5f);
//        ThisThread::sleep_for(3s);
    }
}
