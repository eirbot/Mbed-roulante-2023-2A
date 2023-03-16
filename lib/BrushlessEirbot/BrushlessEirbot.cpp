//#include "BrushlessEirbot.hpp"
//
//void BrushlessEirbot::setDutyCycle() {
//    if (position == Left){
//        if ((data.cmdsat >= 0) && (data.cmdsat <= DC_MAX))			//clockwise
//        {
//            PWM.sense=1;	TIM1->CCR1=25; TIM1->CCR2=35; TIM1->CCR3=45;
//        }
//        else if ((data.cmdsat >= DC_MIN) && (data.cmdsat < 0))	//anti clockwise
//        {
//            PWM.sense=0;	TIM1->CCR1=-data.cmdsat; TIM1->CCR2=-data.cmdsat; TIM1->CCR3=-data.cmdsat;
//        }
//        else														//error state -. PWM off
//        {
//            PWM.sense=1;	TIM1->CCR1=0;	TIM1->CCR2=0;	TIM1->CCR3=0;
//        }
//    } else if(position == Right){
//        if ((data.cmdsat >= 0) && (data.cmdsat <= DC_MAX))  {          //clockwise
//
//            PWM.sense = 1;
//            TIM8->CCR1 = data.cmdsat;
//            TIM8->CCR2 = data.cmdsat;
//            TIM8->CCR3 = data.cmdsat;
//        } else if ((data.cmdsat >= DC_MIN) && (data.cmdsat < 0))    //anti clockwise
//        {
//            PWM.sense = 0;
//            TIM8->CCR1 = -data.cmdsat;
//            TIM8->CCR2 = -data.cmdsat;
//            TIM8->CCR3 = -data.cmdsat;
//        } else                                                        //error state --> PWM off
//        {
//            PWM.sense = 1;
//            TIM8->CCR1 = 0;
//            TIM8->CCR2 = 0;
//            TIM8->CCR3 = 0;
//        }
//    }
//}
//
////void BrushlessEirbot::decodeHall(hall_t *hall, PWM_t *PWM) {
////    /*
////    Commutation sequence for MAXON EC45 BLDC MOTOR
////    */
////
////    hall->h123 = (hall->h1 << 2) | (hall->h2 << 1) | hall->h3; //for motor sense reading
////
////    if (PWM->sense) //clockwise
////    {
////        PWM->aH = (hall->h1) && (!hall->h2);
////        PWM->aL = (!hall->h1) && (hall->h2);
////        PWM->bH = (hall->h2) && (!hall->h3);
////        PWM->bL = (!hall->h2) && (hall->h3);
////        PWM->cH = (!hall->h1) && (hall->h3);
////        PWM->cL = (hall->h1) && (!hall->h3);
////    } else //anti-clockwise
////    {
////        PWM->aH = (!hall->h1) && (hall->h2);
////        PWM->aL = (hall->h1) && (!hall->h2);
////        PWM->bH = (!hall->h2) && (hall->h3);
////        PWM->bL = (hall->h2) && (!hall->h3);
////        PWM->cH = (hall->h1) && (!hall->h3);
////        PWM->cL = (!hall->h1) && (hall->h3);
////    }
////
////    if (hall->h123 != hall->prev_h123) { //for sense sign reading
////        switch (hall->h123) // 101 -> 100 -> 110 -> 010 -> 011 -> 001 clockwise
////        {
////            case 1:
////                if (hall->prev_h123 == 3) (hall->tickS)++;    //clockwise
////                else (hall->tickS)--;
////                break;                //anti clockwise
////            case 2:
////                if (hall->prev_h123 == 6) (hall->tickS)++;
////                else (hall->tickS)--;
////                break;
////            case 3:
////                if (hall->prev_h123 == 2) (hall->tickS)++;
////                else (hall->tickS)--;
////                break;
////            case 4:
////                if (hall->prev_h123 == 5) (hall->tickS)++;
////                else (hall->tickS)--;
////                break;
////            case 5:
////                if (hall->prev_h123 == 1) (hall->tickS)++;
////                else (hall->tickS)--;
////                break;
////            case 6:
////                if (hall->prev_h123 == 4) (hall->tickS)++;
////                else (hall->tickS)--;
////                break;
////            default:
////                break;
////        }
////    }
////    hall->prev_h123 = hall->h123;
////}
////
////// danger changer le timer
////void BrushlessEirbot::updateOutput(PWM_t PWM) {
////    if (PWM.aH) { TIM1->CCER |= TIM_CCER_CC1E; }   //CC1E=1
////    else { TIM1->CCER &= ~(TIM_CCER_CC1E); }   //CC1E=0
////
////    if (PWM.aL) { TIM1->CCER |= TIM_CCER_CC1NE; }    //CC1NE=1
////    else { TIM1->CCER &= ~(TIM_CCER_CC1NE); }   //CC1NE=0
////
////    if (PWM.bH) { TIM1->CCER |= TIM_CCER_CC2E; }
////    else { TIM1->CCER &= ~(TIM_CCER_CC2E); }
////
////    if (PWM.bL) { TIM1->CCER |= TIM_CCER_CC2NE; }
////    else { TIM1->CCER &= ~(TIM_CCER_CC2NE); }
////
////    if (PWM.cH) { TIM1->CCER |= TIM_CCER_CC3E; }
////    else { TIM1->CCER &= ~(TIM_CCER_CC3E); }
////
////    if (PWM.cL) { TIM1->CCER |= TIM_CCER_CC3NE; }
////    else { TIM1->CCER &= ~(TIM_CCER_CC3NE); }
////}
////
////int16_t BrushlessEirbot::calculateSpeed(hall_t *hall) {
////    /*
////     * Calculate motor speed using hall effect sensor
////     * called every 10ms (100Hz)
////     * return measurement_L.speed in tick/s
////     * resolution : 1/p mechanical turn  -> 48ticks for 1turn
////     */
////    if (hall->tickS > 240)
////        return (int16_t) MAX_SPEED;
////    else if (hall->tickS < -240)
////        return (int16_t) MIN_SPEED;
////    else
////        return 100 * hall->tickS;
////}
////
////void BrushlessEirbot::writeCommand(PWM_t *PWM, dataSpeed_t *data) {
////    if ((data->cmdsat >= 0) && (data->cmdsat <= DC_MAX))            //clockwise
////    {
////        PWM->sense = 1;
////        TIM1->CCR1 = data->cmdsat;
////        TIM1->CCR2 = data->cmdsat;
////        TIM1->CCR3 = data->cmdsat;
////    } else if ((data->cmdsat >= DC_MIN) && (data->cmdsat < 0))    //anti clockwise
////    {
////        PWM->sense = 0;
////        TIM1->CCR1 = -data->cmdsat;
////        TIM1->CCR2 = -data->cmdsat;
////        TIM1->CCR3 = -data->cmdsat;
////    } else                                                        //error state --> PWM off
////    {
////        PWM->sense = 1;
////        TIM1->CCR1 = 0;
////        TIM1->CCR2 = 0;
////        TIM1->CCR3 = 0;
////    }
////}
//
//BrushlessEirbot::BrushlessEirbot(position_t position_motor) {
//    hall = {.h1=0, .h2=0, .h3=0, .h123=0, .prev_h123=0, .tickS=0, .tickP=0};
//    PWM = {.aH=1, .aL=0, .bH=1, .bL=0, .cH=0, .cL=0, .sense=1};
//    data = {.speed_ref=0, .speed=0, .error=0, .prev_error=0, .cmd=0, .prev_cmd=0, .cmdsat=10, .prev_cmdsat=0};
//    position = position_motor;
//    if (position == Left) {
//        // Configuration des Hall Sensors
////        pin_HALL_1 = PB_2;
////        pin_HALL_2 = PB_5;
////        pin_HALL_3 = PB_6;
//
//        // Configuration des Current Sensors
////        pin_Current_A = PC_0;
////        pin_Current_B = PC_1;
////        pin_Current_C = PC_2;
//
//        // Configuration du timer 1
//        MX_TIM1_Init();
//    }
////    else if (position == Right) {
////        // Configuration des Hall Sensors
//////        pin_HALL_1 = PB_7;
//////        pin_HALL_2 = PB_8;
//////        pin_HALL_3 = PB_9;
////
////        // Configuration des Current Sensors
//////        pin_Current_A = PC_3;
//////        pin_Current_B = PC_4;
//////        pin_Current_C = PC_5;
////
////        // Configuration du timer 8
////        MX_TIM8_Init();
////    }
//    // Pins
////    HALL_1 = new InterruptIn(pin_HALL_1);
////    HALL_2 = new InterruptIn(pin_HALL_2);
////    HALL_3 = new InterruptIn(pin_HALL_3);
////    Current_A = new AnalogIn(pin_Current_A);
////    Current_B = new AnalogIn(pin_Current_B);
////    Current_C = new AnalogIn(pin_Current_C);
//}
//
//BrushlessEirbot::~BrushlessEirbot() {
////    delete HALL_1;
////    delete HALL_2;
////    delete HALL_3;
////    delete Current_A;
////    delete Current_B;
////    delete Current_C;
//}
//
//
//
//
