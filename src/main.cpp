// ============= INCLUDES =================
#include "mbed.h"
#include "tim.h"
// ============= DEFINITIONS =================

DigitalOut led(LED1);



// ============= MAIN =================
int main() {
//    motor.setDutyCycle();
    led.write(0);

    MX_TIM1_Init();

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    TIM1->CCR1=25; TIM1->CCR2=35; TIM1->CCR3=45;
    led.write(1);



    while (true) {
        led = !led;
        ThisThread::sleep_for(1s);
    }

}