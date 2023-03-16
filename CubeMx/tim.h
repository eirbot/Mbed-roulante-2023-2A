#ifndef __TIM_H__
#define __TIM_H__

#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AL_R_Pin GPIO_PIN_7
#define AL_R_GPIO_Port GPIOA
#define BL_L_Pin GPIO_PIN_0
#define BL_L_GPIO_Port GPIOB
#define CL_L_Pin GPIO_PIN_1
#define CL_L_GPIO_Port GPIOB
#define AL_L_Pin GPIO_PIN_13
#define AL_L_GPIO_Port GPIOB
#define BL_R_Pin GPIO_PIN_14
#define BL_R_GPIO_Port GPIOB
#define CL_R_Pin GPIO_PIN_15
#define CL_R_GPIO_Port GPIOB
#define AH_R_Pin GPIO_PIN_6
#define AH_R_GPIO_Port GPIOC
#define BH_R_Pin GPIO_PIN_7
#define BH_R_GPIO_Port GPIOC
#define CH_R_Pin GPIO_PIN_8
#define CH_R_GPIO_Port GPIOC
#define AH_L_Pin GPIO_PIN_8
#define AH_L_GPIO_Port GPIOA
#define BH_L_Pin GPIO_PIN_9
#define BH_L_GPIO_Port GPIOA
#define CH_L_Pin GPIO_PIN_10
#define CH_L_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13


void Error_Handler(void);
void SystemClock_Config(void);
/*
 ****************  tim  ****************   
 */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;


void MX_TIM1_Init(void);
void MX_TIM8_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

