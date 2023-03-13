//#include "BrushlessEirbot.hpp"
//
//#define debug
//
//
//void Error_Handler(void) {
//    __disable_irq();
//    while (1) {}
//}
//
///************************************************************************************
// *  ========================= Constructeurs & Destructeurs ======================== *
// ************************************************************************************/
//BrushlessEirbot::BrushlessEirbot(position_t position_motor, struct PID_t PID) {
//#ifdef debug
//    std::cout << "Constructeur" << std::endl;
//#endif
//#ifdef debug
//    std::cout << "Initialisation du timer pour les PWMs" << std::endl;
//#endif
//    void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
//
//    hall = {.h1=0, .h2=0, .h3=0, .h123=0, .prev_h123=0, .tickS=0, .tickP=0};
//    PWM = {.aH=0, .aL=0, .bH=0, .bL=0, .cH=0, .cL=0, .sense=1};
//    dataSpeed = {.speed_ref=0, .speed=0, .error=0, .prev_error=0, .cmd=0, .cmdsat=0, .prev_cmd=0, .prev_cmdsat=0};
//
//    if (position_motor == Left) {
//        // Configuration des Hall Sensors
//#ifdef debug
//        std::cout << "Configuration des Hall Sensors" << std::endl;
//#endif
//        pin_HALL_1 = PB_2;
//        pin_HALL_2 = PB_5;
//        pin_HALL_3 = PB_6;
//
//        // Configuration des Current Sensors
//#ifdef debug
//        std::cout << "Configuration des Current Sensors" << std::endl;
//#endif
//        pin_Current_A = PC_0;
//        pin_Current_B = PC_1;
//        pin_Current_C = PC_2;
//
//        // Configuration du timer 1
//#ifdef debug
//        std::cout << "Configuration du timer 1" << std::endl;
//#endif
//        TIM_HandleTypeDef htim1;
//        TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//        TIM_MasterConfigTypeDef sMasterConfig = {0};
//        TIM_OC_InitTypeDef sConfigOC = {0};
//        TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
//        htim1.Instance = TIM1;
//        htim1.Init.Prescaler = 28 - 1;
//        htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//        htim1.Init.Period = 100 - 1;
//        htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//        htim1.Init.RepetitionCounter = 0;
//        htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//        if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
//            Error_Handler();
//        }
//        sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//        if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
//            Error_Handler();
//        }
//        if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
//            Error_Handler();
//        }
//        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//        if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
//            Error_Handler();
//        }
//        sConfigOC.OCMode = TIM_OCMODE_PWM1;
//        sConfigOC.Pulse = 0;
//        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//        sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
//        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//        sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
//        sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//        if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
//            Error_Handler();
//        }
//        if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
//            Error_Handler();
//        }
//        if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
//            Error_Handler();
//        }
//        sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
//        sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
//        sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
//        sBreakDeadTimeConfig.DeadTime = 2;
//        sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
//        sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
//        sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
//        if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
//            Error_Handler();
//        }
//
//        HAL_TIM_MspPostInit(&htim1); // DANGER
//    } else if (position_motor == Right) {
//        // Configuration des Hall Sensors
//#ifdef debug
//        std::cout << "Configuration des Hall Sensors" << std::endl;
//#endif
//        pin_HALL_1 = PB_7;
//        pin_HALL_2 = PB_8;
//        pin_HALL_3 = PB_9;
//
//        // Configuration des Current Sensors
//#ifdef debug
//        std::cout << "Configuration des Current Sensors" << std::endl;
//#endif
//        pin_Current_A = PC_3;
//        pin_Current_B = PC_4;
//        pin_Current_C = PC_5;
//
//        // Configuration du timer 8
//#ifdef debug
//        std::cout << "Configuration du timer 8" << std::endl;
//#endif
//        TIM_HandleTypeDef htim8;
//        TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//        TIM_MasterConfigTypeDef sMasterConfig = {0};
//        TIM_OC_InitTypeDef sConfigOC = {0};
//        TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
//        htim8.Instance = TIM8;
//        htim8.Init.Prescaler = 28 - 1;
//        htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
//        htim8.Init.Period = 100 - 1;
//        htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//        htim8.Init.RepetitionCounter = 0;
//        htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//        if (HAL_TIM_Base_Init(&htim8) != HAL_OK) {
//            Error_Handler();
//        }
//        sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//        if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) {
//            Error_Handler();
//        }
//        if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
//            Error_Handler();
//        }
//        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//        if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) {
//            Error_Handler();
//        }
//        sConfigOC.OCMode = TIM_OCMODE_PWM1;
//        sConfigOC.Pulse = 0;
//        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//        sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
//        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//        sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
//        sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//        if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
//            Error_Handler();
//        }
//        if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
//            Error_Handler();
//        }
//        if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
//            Error_Handler();
//        }
//        sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
//        sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
//        sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
//        sBreakDeadTimeConfig.DeadTime = 2;
//        sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
//        sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
//        sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
//        if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK) {
//            Error_Handler();
//        }
//
//        HAL_TIM_MspPostInit(&htim8); // DANGER
//    }
//    // Pins
//    HALL_1 = new InterruptIn(pin_HALL_1);
//    HALL_2 = new InterruptIn(pin_HALL_2);
//    HALL_3 = new InterruptIn(pin_HALL_3);
////    Current_A = new AnalogIn(pin_Current_A);
////    Current_B = new AnalogIn(pin_Current_B);
////    Current_C = new AnalogIn(pin_Current_C);
//}
//
//BrushlessEirbot::~BrushlessEirbot() {
//    delete HALL_1;
//    delete HALL_2;
//    delete HALL_3;
////    delete Current_A;
////    delete Current_B;
////    delete Current_C;
//}
//
///************************************************************************************
// *  ================================    Méthodes   ================================ *
// ************************************************************************************/
//void BrushlessEirbot::setDutyCycle(struct PWM_t *PWM, struct dataSpeed_t *data) {
//    if ((data->cmdsat >= 0) && (data->cmdsat <= DC_MAX))            //clockwise
//    {
//        PWM->sense = 1;
//        TIM8->CCR1 = data->cmdsat;
//        TIM8->CCR2 = data->cmdsat;
//        TIM8->CCR3 = data->cmdsat;
//    } else if ((data->cmdsat >= DC_MIN) && (data->cmdsat < 0))    //anti clockwise
//    {
//        PWM->sense = 0;
//        TIM8->CCR1 = -data->cmdsat;
//        TIM8->CCR2 = -data->cmdsat;
//        TIM8->CCR3 = -data->cmdsat;
//    } else                                                        //error state --> PWM off
//    {
//        PWM->sense = 1;
//        TIM8->CCR1 = 0;
//        TIM8->CCR2 = 0;
//        TIM8->CCR3 = 0;
//    }
//}
//
//void BrushlessEirbot::decodeHall(struct hall_t *hall, struct PWM_t *PWM) {
//    /*
//    Commutation sequence for MAXON EC45 BLDC MOTOR
//    */
//
//    hall->h123 = (hall->h1 << 2) | (hall->h2 << 1) | hall->h3; //for motor sense reading
//
//    if (PWM->sense) //clockwise
//    {
//        PWM->aH = (hall->h1) && (!hall->h2);
//        PWM->aL = (!hall->h1) && (hall->h2);
//        PWM->bH = (hall->h2) && (!hall->h3);
//        PWM->bL = (!hall->h2) && (hall->h3);
//        PWM->cH = (!hall->h1) && (hall->h3);
//        PWM->cL = (hall->h1) && (!hall->h3);
//    } else //anti-clockwise
//    {
//        PWM->aH = (!hall->h1) && (hall->h2);
//        PWM->aL = (hall->h1) && (!hall->h2);
//        PWM->bH = (!hall->h2) && (hall->h3);
//        PWM->bL = (hall->h2) && (!hall->h3);
//        PWM->cH = (hall->h1) && (!hall->h3);
//        PWM->cL = (!hall->h1) && (hall->h3);
//    }
//
//    if (hall->h123 != hall->prev_h123) { //for sense sign reading
//        switch (hall->h123) // 101 -> 100 -> 110 -> 010 -> 011 -> 001 clockwise
//        {
//            case 1:
//                if (hall->prev_h123 == 3) (hall->tickS)++;    //clockwise
//                else (hall->tickS)--;
//                break;                //anti clockwise
//            case 2:
//                if (hall->prev_h123 == 6) (hall->tickS)++;
//                else (hall->tickS)--;
//                break;
//            case 3:
//                if (hall->prev_h123 == 2) (hall->tickS)++;
//                else (hall->tickS)--;
//                break;
//            case 4:
//                if (hall->prev_h123 == 5) (hall->tickS)++;
//                else (hall->tickS)--;
//                break;
//            case 5:
//                if (hall->prev_h123 == 1) (hall->tickS)++;
//                else (hall->tickS)--;
//                break;
//            case 6:
//                if (hall->prev_h123 == 4) (hall->tickS)++;
//                else (hall->tickS)--;
//                break;
//            default:
//                break;
//        }
//    }
//    hall->prev_h123 = hall->h123;
//}
//
//// danger changer le timer
//void BrushlessEirbot::updateOutput(struct PWM_t PWM) {
//    if (PWM.aH) { TIM1->CCER |= TIM_CCER_CC1E; }   //CC1E=1
//    else { TIM1->CCER &= ~(TIM_CCER_CC1E); }   //CC1E=0
//
//    if (PWM.aL) { TIM1->CCER |= TIM_CCER_CC1NE; }    //CC1NE=1
//    else { TIM1->CCER &= ~(TIM_CCER_CC1NE); }   //CC1NE=0
//
//    if (PWM.bH) { TIM1->CCER |= TIM_CCER_CC2E; }
//    else { TIM1->CCER &= ~(TIM_CCER_CC2E); }
//
//    if (PWM.bL) { TIM1->CCER |= TIM_CCER_CC2NE; }
//    else { TIM1->CCER &= ~(TIM_CCER_CC2NE); }
//
//    if (PWM.cH) { TIM1->CCER |= TIM_CCER_CC3E; }
//    else { TIM1->CCER &= ~(TIM_CCER_CC3E); }
//
//    if (PWM.cL) { TIM1->CCER |= TIM_CCER_CC3NE; }
//    else { TIM1->CCER &= ~(TIM_CCER_CC3NE); }
//}
//
//int16_t BrushlessEirbot::calculateSpeed(struct hall_t *hall) {
//    /*
//     * Calculate motor speed using hall effect sensor
//     * called every 10ms (100Hz)
//     * return measurement_L.speed in tick/s
//     * resolution : 1/p mechanical turn  -> 48ticks for 1turn
//     */
//    if (hall->tickS > 240)
//        return (int16_t) MAX_SPEED;
//    else if (hall->tickS < -240)
//        return (int16_t) MIN_SPEED;
//    else
//        return 100 * hall->tickS;
//}
//
//void BrushlessEirbot::writeCommand(struct PWM_t *PWM, struct dataSpeed_t *data) {
//    if ((data->cmdsat >= 0) && (data->cmdsat <= DC_MAX))            //clockwise
//    {
//        PWM->sense = 1;
//        TIM1->CCR1 = data->cmdsat;
//        TIM1->CCR2 = data->cmdsat;
//        TIM1->CCR3 = data->cmdsat;
//    } else if ((data->cmdsat >= DC_MIN) && (data->cmdsat < 0))    //anti clockwise
//    {
//        PWM->sense = 0;
//        TIM1->CCR1 = -data->cmdsat;
//        TIM1->CCR2 = -data->cmdsat;
//        TIM1->CCR3 = -data->cmdsat;
//    } else                                                        //error state --> PWM off
//    {
//        PWM->sense = 1;
//        TIM1->CCR1 = 0;
//        TIM1->CCR2 = 0;
//        TIM1->CCR3 = 0;
//    }
//}
//
//
//
//
