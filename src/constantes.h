#ifndef MBED_CONSTANTES_H
#define MBED_CONSTANTES_H

#define PIN_CANON	        PB_10
#define PIN_DANGER		    PB_12
#define PIN_AVERTISSEMENT	PA_12
#define TX_LIDAR	    	PD_2
#define RX_LIDAR	    	PC_12
#define PIN_PINCE_SERVOS_1	PA_0
#define PIN_PINCE_SERVOS_2	PA_5
#define PIN_FUNNY_SERVOS	PA_4
#define PIN_FDC_BL          PA_15
#define PIN_FDC_BR	        PB_4
#define PIN_FDC_FL		    PA_6
#define PIN_FDC_FR	    	PA_11
#define PIN_TIRETTE	        PA_1


// Motor MBED and HALL sensor config
#define MOTOR_UPDATE_RATE 10ms // 10ms for 100Hz
#define MOTOR_FLAG 0x01
#define ENC_HALL_RESOLUTION 256 // max uint8
#define MOTOR_RESOLUTION 48
#define MOTOR_REDUCTION 14
#define WHEEL_RESOLUTION float(MOTOR_RESOLUTION*MOTOR_REDUCTION)
#define WHEEL_RADIUS (0.080f/2.0f)
#define MAX_PWM 0.7f
#define WHEELS_DISTANCE 0.30f

#define PID_TETA_PRECISION  0.0872665f // 5°
#define PID_DV_PRECISION 0.03f // 3 cm





#endif //MBED_CONSTANTES_H
