// ============= INCLUDES =================
#include "mbed.h"
#include "odometry_eirbot.h"
#include "motor_base_eirbot.h"
#include "rbdc.h"
#include "motor_brushless_eirbot.h"
#include "odometry_eirbot.h"
#include "constantes.h"

// ============= Définition =================
DigitalIn fdc_fl(PIN_FDC_FL, PullDown);
DigitalIn fdc_fr(PIN_FDC_FR, PullDown);
DigitalIn fdc_bl(PIN_FDC_BL, PullDown);
DigitalIn fdc_br(PIN_FDC_BR, PullDown);

DigitalIn danger_lidar(PIN_DANGER);
DigitalIn avertissement_lidar(PIN_AVERTISSEMENT);

DigitalIn tirette(PIN_TIRETTE, PullDown);

DigitalOut deguisement(PIN_FUNNY_SERVOS);
DigitalOut canon(PIN_CANON);
DigitalOut pince1(PIN_PINCE_SERVOS_1);
DigitalOut pince2(PIN_PINCE_SERVOS_2);
DigitalOut led(LED1);


// Motor MBED and HALL sensorconfig
Ticker MotorUpdateTicker;
EventFlags MotorFlag;
Thread motorThread(osPriorityRealtime);
sixtron::MotorBaseEirbot *base_eirbot;
sixtron::MotorBrushlessEirbot *motor_left;
sixtron::MotorBrushlessEirbot *motor_right;
bool motor_init_done = false;
// Odometry
sixtron::OdometryEirbot *odom;
// RBDC
sixtron::RBDC *rbdc_eirbot;
sixtron::RBDC_params rbdc_eirbot_params;
float robot_target_X = 0.0f, robot_target_Y = 0.0f, robot_target_theta = 0.0f;
int rbdc_result;


int16_t compteur_lidar;

bool security() {
    if (tirette.read() != 1) {
        if (danger_lidar.read() || avertissement_lidar.read()) {
            if (tirette.read() != 1) {
                compteur_lidar++;
            }
        }
        if (compteur_lidar == 5) {
            return true;
        } else {
            return false;
        }
    }
}



//// incrément décrément
//if (tirette.read() != 1) {
//if (danger_lidar.read() || avertissement_lidar.read()) {
//if (tirette.read() != 1 ) {
//compteur_lidar++;
//}
//}
//
//if (compteur_lidar < 0) {
//compteur_lidar = 0;
//return false;
//} else if (compteur_lidar == 5) {
//return true;
//} else if (compteur_lidar > 10) {
//compteur_lidar = 5;
//return true;
//} else {
//return false;
//}
//}
//return false;

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

        // SECRITY
//        bool secu = security();
//        if (secu) {
//            rbdc_eirbot->stop();
//        }


        // Update Target
        target_pos.x = robot_target_X;
        target_pos.y = robot_target_Y;
        target_pos.theta = robot_target_theta;
        rbdc_eirbot->setTarget(target_pos);

        // Update RBDC (this will update odometry and motor base)
        rbdc_result = rbdc_eirbot->update();
    }
}


//BufferedSerial pc(USBTX, USBRX, 115200);

//void print(std::string str){
//    pc.write(str.c_str(), str.length());
//}






int main() {
    std::string str;

    pince1.write(0);
    pince2.write(1);

    // Start the thread for motor control
    motorThread.start(motorThreadMain);
    // Setup ticker to update the motor base flag at exactly the defined rate
    MotorUpdateTicker.attach(&MotorFlagUpdate, MOTOR_UPDATE_RATE);
    // Waiting for the motor to be setup ...
    while (!motor_init_done);
    ThisThread::sleep_for(1500ms);


    while (tirette.read());

    robot_target_X = 0.35f;
    robot_target_Y = 0.0f;
    robot_target_theta = 0.0f;
    while (rbdc_result != sixtron::RBDC_status::RBDC_done);


    ThisThread::sleep_for(2s);

    pince1.write(0);
    pince2.write(1);
    ThisThread::sleep_for(3s);

    robot_target_X = 0.1f;
    robot_target_Y = 0.0f;
    robot_target_theta = 0.0f;
    while (rbdc_result != sixtron::RBDC_status::RBDC_done);
    ThisThread::sleep_for(3s);


//    pince1.write(1);
//    pince2.write(0);
//
//    ThisThread::sleep_for(4s);
//
//    pince1.write(1);
//    pince2.write(1);
////
//    robot_target_X = 0.15f;
//    robot_target_Y = 0.0f;
//    robot_target_theta = 0.0f;
//    while (rbdc_result != sixtron::RBDC_status::RBDC_done);
//    ThisThread::sleep_for(2s);
//
//    pince1.write(0);
//    pince2.write(0);
//
//    ThisThread::sleep_for(2s);
//

//    robot_target_X = 0.50f;
//    robot_target_Y = 0.0f;
//    robot_target_theta = 0.0f;
//    while (rbdc_result != sixtron::RBDC_status::RBDC_done);
//    ThisThread::sleep_for(2s);

    while (1);
}
