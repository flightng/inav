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
#include "drivers/sensor.h"


BUSDEV_REGISTER_SPI_TAG(busdev_bmi088_gyro, DEVHW_BMI088_GYRO,  BMI088_SPI_BUS,     BMI088_GYRO_CS_PIN, NONE,       0,  DEVFLAGS_NONE,  IMU_BMI088_ALIGN);
BUSDEV_REGISTER_SPI_TAG(busdev_bmi088_acc,  DEVHW_BMI088_ACC,   BMI088_SPI_BUS,     BMI088_ACC_CS_PIN,  NONE,       0,  DEVFLAGS_NONE,  IMU_BMI088_ALIGN);
BUSDEV_REGISTER_SPI_TAG(busdev_icm42605,    DEVHW_ICM42605,     ICM42605_SPI_BUS,   ICM42605_CS_PIN,    NONE,       1,  DEVFLAGS_NONE,  IMU_ICM42605_ALIGN);
BUSDEV_REGISTER_SPI_TAG(busdev_bmi270,      DEVHW_BMI270,       BMI270_SPI_BUS,     BMI270_CS_PIN,      NONE,       1, DEVFLAGS_NONE,  IMU_BMI270_ALIGN);


timerHardware_t timerHardware[] = {

    // DEF_TIM(TMR5, CH4, PA3,  TIM_USE_PPM,  0, 4), 						// PPM UART2_RX_PIN DMA1 CH5

 	DEF_TIM(TMR4, CH1, PB6,  TIM_USE_MOTOR, 0,0),  // motor1 DMA1 CH1
 	DEF_TIM(TMR4, CH2, PB7,  TIM_USE_MOTOR, 0,1),  // motor2 DMA1 CH2
 	DEF_TIM(TMR4, CH3, PB8,  TIM_USE_MOTOR, 0,2), // motor3 DMA1 CH3
 	DEF_TIM(TMR4, CH4, PB9,  TIM_USE_MOTOR, 0,3), // motor4 DMA1 CH4

	DEF_TIM(TMR8, CH1, PC6,  TIM_USE_OUTPUT_AUTO, 0,8), 		// PWM1 - OUT5  DMA2 CH2 DMA2_CHANNEL1->ADC
	DEF_TIM(TMR8, CH2, PC7,  TIM_USE_OUTPUT_AUTO, 0,9), 		// PWM2 - OUT6  DMA2 CH3
	DEF_TIM(TMR8, CH3, PC8,  TIM_USE_OUTPUT_AUTO, 0,10), 		// PWM3 - OUT7  DMA2 CH4
	DEF_TIM(TMR8, CH4, PC9,  TIM_USE_OUTPUT_AUTO, 0,11), 		// PWM4 - OUT8  DMA2 CH5

    DEF_TIM(TMR3, CH1, PB4,   TIM_USE_OUTPUT_AUTO, 0,5), 		// PWM5 - OUT  NO DMA
	DEF_TIM(TMR3, CH2, PB5,   TIM_USE_OUTPUT_AUTO, 0,6), 		// PWM6 - OUT  NO DMA
	DEF_TIM(TMR5, CH4, PB11,  TIM_USE_SERVO, 0,0), 		// PWM7 - OUT  NO DMA
	DEF_TIM(TMR5, CH1, PB12,  TIM_USE_SERVO, 0,0), 		// PWM8 - OUT  NO DMA

    DEF_TIM(TMR1, CH1, PE9 ,  TIM_USE_SERVO, 0,0), 		// PWM9 -  OUT  NO DMA
	DEF_TIM(TMR1, CH2, PE11,  TIM_USE_SERVO, 0,0), 		// PWM10 - OUT  NO DMA
	DEF_TIM(TMR1, CH3, PE13,  TIM_USE_SERVO, 0,0), 		// PWM11 - OUT  NO DMA
	DEF_TIM(TMR1, CH4, PE14,  TIM_USE_SERVO, 0,0), 		// PWM12 - OUT  NO DMA

    DEF_TIM(TMR2, CH3, PB10, TIM_USE_ANY | TIM_USE_LED, 0,13),           //LED MCO1 NO_DMA
	// DEF_TIM(TMR5, CH4, PC13, TIM_USE_ANY | TIM_USE_BEEPER, 0,0), 		 //BB NO_DMA,PWM_MODE=OFF NOUSE TIMER
};

const int timerHardwareCount = sizeof(timerHardware) / sizeof(timerHardware[0]);
  
