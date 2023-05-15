#ifndef MBED_CONSTANTES_H
#define MBED_CONSTANTES_H

#define PIN_CANON	        PB_10
#define PIN_DANGER		    PB_12
#define PIN_AVERTISSEMENT	PA_12
#define TX_LIDAR	    	PD_2
#define RX_LIDAR	    	PC_12
#define PIN_PINCE_SERVOS	PA_0
#define PIN_FUNNY_SERVOS	PA_4
#define PIN_FDC_FL          PA_15
#define PIN_FDC_FR	        PB_4
#define PIN_FDC_BL		    PA_6
#define PIN_FDC_BR	    	PA_11
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





#endif //MBED_CONSTANTES_H
