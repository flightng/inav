/*
* This file is part of Cleanflight.
*
* Cleanflight is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Cleanflight is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdint.h>

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"

timerHardware_t timerHardware[] = {

    //DEF_TIM(TMR5, CH4, PA3, TIM_USE_PPM,                         0, 0),                     // PPM
    DEF_TIM(TMR2, CH3, PB10, TIM_USE_ANY |TIM_USE_LED, 0,0), // PWM1 - LED MCO1 DMA1 CH2
	DEF_TIM(TMR2, CH4, PB11,  TIM_USE_ANY |TIM_USE_BEEPER, 0,0), // PWM2 - BB DMA1 CH6

	DEF_TIM(TMR3, CH1, PC6,  TIM_USE_ANY |TIM_USE_FW_MOTOR, 0,0), // PWM3 - OUT3  DMA1 CH7
	DEF_TIM(TMR3, CH2, PC7,  TIM_USE_ANY |TIM_USE_FW_MOTOR, 0,0), // PWM4 - OUT4  DMA2 CH1
	DEF_TIM(TMR3, CH3, PC8,  TIM_USE_ANY |TIM_USE_FW_MOTOR, 0,0), // PWM5 - OUT5  DMA2 CH2
	DEF_TIM(TMR3, CH4, PC9,  TIM_USE_ANY |TIM_USE_FW_MOTOR, 0,0), // PWM6 - OUT6  DMA2 CH3

 	DEF_TIM(TMR4, CH1, PB6,  TIM_USE_MC_MOTOR, 0,0),  // motor1 DMA2 CH7
 	DEF_TIM(TMR4, CH2, PB7,  TIM_USE_MC_MOTOR, 0,0),  // motor2 DMA2 CH6
 	DEF_TIM(TMR4, CH3, PB8,  TIM_USE_MC_MOTOR,  0,0), // motor3 DMA2 CH5
 	DEF_TIM(TMR4, CH4, PB9,  TIM_USE_MC_MOTOR,  0,0), // motor4 DMA2 CH4

};

const int timerHardwareCount = sizeof(timerHardware) / sizeof(timerHardware[0]);
  