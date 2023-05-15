// ============= INCLUDES =================
#include "mbed.h"
<<<<<<< Updated upstream
#include "odometry_eirbot.h"
#include "motor_base_eirbot.h"
#include "rbdc.h"
=======
#include "motor_brushless_eirbot.h"
#include "odometry_eirbot.h"
#include "constantes.h"
>>>>>>> Stashed changes

// ============= Définition =================
InterruptIn fdc_fl(PIN_FDC_FL, PullDown);
InterruptIn fdc_fr(PIN_FDC_FR, PullDown);
InterruptIn fdc_bl(PIN_FDC_BL, PullDown);
InterruptIn fdc_br(PIN_FDC_BR, PullDown);

InterruptIn danger_lidar(PIN_DANGER);
InterruptIn avertissement_lidar(PIN_AVERTISSEMENT);

DigitalIn tirette(PIN_TIRETTE);

DigitalOut deguisement(PIN_FUNNY_SERVOS);
DigitalOut canon(PIN_CANON);
DigitalOut pince(PIN_PINCE_SERVOS);
DigitalOut led(LED1);

<<<<<<< Updated upstream
// Motor MBED and HALL sensorconfig
#define MOTOR_UPDATE_RATE 10ms // 10ms for 100Hz
#define MOTOR_FLAG 0x01
Ticker MotorUpdateTicker;
EventFlags MotorFlag;
Thread motorThread(osPriorityRealtime);
sixtron::MotorBaseEirbot *base_eirbot;
=======
// ============= TUDOR =================
Ticker MotorUpdateTicker;
EventFlags MotorFlag;
Thread motorThread(osPriorityRealtime);

sixtron::MotorBrushlessEirbot *motor_left;
sixtron::MotorBrushlessEirbot *motor_right;
>>>>>>> Stashed changes
bool motor_init_done = false;

// Odometry
sixtron::OdometryEirbot *odom;

// RBDC
#define PID_TETA_PRECISION  0.0872665f // 5°
#define PID_DV_PRECISION 0.03f // 3 cm
sixtron::RBDC *rbdc_eirbot;
sixtron::RBDC_params rbdc_eirbot_params;
float robot_target_X = 0.0f, robot_target_Y = 0.0f, robot_target_theta = 0.0f;
int rbdc_result;

void MotorFlagUpdate() {
    MotorFlag.set(MOTOR_FLAG);
}

void motorThreadMain() {
    // First, convert the rate of the loop in seconds [float]
    auto f_secs = std::chrono::duration_cast<std::chrono::duration<float>>(
            MOTOR_UPDATE_RATE);
    float dt_pid = f_secs.count();
    float rate_hz = dt_pid / 1.0f;

    // Init motor base. This will create and init both brushless motors + hall sensors.
    base_eirbot = new sixtron::MotorBaseEirbot(dt_pid);
    base_eirbot->init();

    // Init odometry
    odom = new sixtron::OdometryEirbot(rate_hz,
                                       base_eirbot,
                                       WHEEL_RESOLUTION,
                                       WHEEL_RADIUS,
                                       WHEELS_DISTANCE);
    odom->init();

    // Init RBDC
    rbdc_eirbot_params.rbdc_format = sixtron::RBDC_format::two_wheels_robot;
    rbdc_eirbot_params.max_output_dv = MAX_PWM - 0.3f;
    rbdc_eirbot_params.max_output_dtheta = MAX_PWM + 0.2f;
    rbdc_eirbot_params.can_go_backward = true;
    rbdc_eirbot_params.dt_seconds = dt_pid;
    rbdc_eirbot_params.final_theta_precision = PID_TETA_PRECISION;
    rbdc_eirbot_params.moving_theta_precision = 2 * PID_TETA_PRECISION;
    rbdc_eirbot_params.target_precision = 2.5f * PID_DV_PRECISION;
    rbdc_eirbot_params.dv_precision = PID_DV_PRECISION;

    rbdc_eirbot_params.pid_param_dteta.Kp = 7.0f;
    rbdc_eirbot_params.pid_param_dteta.Ki = 0.5f;
    rbdc_eirbot_params.pid_param_dteta.Kd = 0.0f;

    rbdc_eirbot_params.pid_param_dv.Kp = 0.8f;
    rbdc_eirbot_params.pid_param_dv.Ki = 0.01f;
    rbdc_eirbot_params.pid_param_dv.Kd = 0.0f;
    rbdc_eirbot_params.pid_param_dv.ramp = 0.4f * dt_pid;

    rbdc_eirbot = new sixtron::RBDC(odom,
                                    base_eirbot,
                                    rbdc_eirbot_params); // will init odom and robot base as well
    sixtron::position target_pos;
    rbdc_eirbot->start();

    // Wait and Done
    ThisThread::sleep_for(500ms);
    motor_init_done = true;

    int printf_debug_incr = 0;
    while (true) {
        // wait for the flag trigger
        MotorFlag.wait_any(MOTOR_FLAG);

        // Update Target
        target_pos.x = robot_target_X;
        target_pos.y = robot_target_Y;
        target_pos.theta = robot_target_theta;
        rbdc_eirbot->setTarget(target_pos);

        // Update RBDC (this will update odometry and motor base)
        rbdc_result = rbdc_eirbot->update();

        // debug
        printf_debug_incr++;
        if (printf_debug_incr > 5) {
            printf_debug_incr = 0;
            printf("Odom: x=%3.2fm, y=%3.2fm, theta=%3.2frad\n",
                   odom->getX(),
                   odom->getY(),
                   odom->getTheta());
        }
    }
}

<<<<<<< Updated upstream
=======
// Just for the debug
void set_motor_target(float speed_left, float speed_right) {
//    printf("Applying %2.3f m/s to the motor.\n", speed_ms);
    motor_left->setSpeed(speed_left);
    motor_right->setSpeed(speed_right);
}
void set_motor_target(float speed_ms) {
//    printf("Applying %2.3f m/s to the motor.\n", speed_ms);
    set_motor_target(speed_ms, speed_ms);
}

>>>>>>> Stashed changes
int main() {
    // Start the thread for motor control
    motorThread.start(motorThreadMain);
    // Setup ticker to update the motor base flag at exactly the defined rate
    MotorUpdateTicker.attach(&MotorFlagUpdate, MOTOR_UPDATE_RATE);

    // Waiting for the motor to be setup ...
    while (!motor_init_done);
<<<<<<< Updated upstream
    printf("Motor init done, continue with setting targets.\n");
    ThisThread::sleep_for(1000ms);
=======
    // Motor init done, continue with setting targets
    ThisThread::sleep_for(500ms);
>>>>>>> Stashed changes

    int square_state = 0;
    while (true) {
<<<<<<< Updated upstream

        // Do the Holy Square indefinitely
        if (rbdc_result == sixtron::RBDC_status::RBDC_done) {
            square_state++;

            switch (square_state) {
                case 1:
                    robot_target_X = 0.0f;
                    robot_target_Y = 0.0f;
                    robot_target_theta = 0.0f;
                    break;
                case 2:
                    robot_target_X = 0.5f;
                    robot_target_Y = 0.0f;
                    robot_target_theta = -1.57f;
                    break;
                case 3:
                    robot_target_X = 0.5f;
                    robot_target_Y = -0.5f;
                    robot_target_theta = -3.14f;
                    break;
                case 4:
                    robot_target_X = 0.0f;
                    robot_target_Y = -0.5f;
                    robot_target_theta = +1.57f;
                    break;
                default:
                    robot_target_X = 0.0f;
                    robot_target_Y = 0.0f;
                    robot_target_theta = 0.0f;
                    square_state = 1;
                    break;
            }
        }


//        robot_target_X = 0.0f;
//        robot_target_Y = 0.0f;
//        robot_target_theta = -2.00f;
//
//        while(rbdc_result != sixtron::RBDC_status::RBDC_done);
//        ThisThread::sleep_for(6s);
//
//        robot_target_X = 0.0f;
//        robot_target_Y = 0.0f;
//        robot_target_theta = 0.0f;
//
//        while(rbdc_result != sixtron::RBDC_status::RBDC_done);
//        ThisThread::sleep_for(6s);



        ThisThread::sleep_for(100ms);
=======
        set_motor_target(+0.3f);
        ThisThread::sleep_for(3s);

        set_motor_target(-0.2f, +0.2f);
        ThisThread::sleep_for(2300ms);
>>>>>>> Stashed changes
    }
}
