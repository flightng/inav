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

#pragma once

#define TARGET_BOARD_IDENTIFIER "NERC"

#define USBD_PRODUCT_STRING  "NeutronRC F435 SE"

/**********swd debuger reserved *****************
 *
 * pa13	swdio
 * pa14 swclk
 * PA15	JTDI
 * PB4 JREST
 * pb3 swo /DTO

 * other pin
 *
 * PB2 ->BOOT0 button
 * PA8  MCO1
 * PA11 OTG1 D+ DP
 * PA10 OTG1 D- DN
 * PH0 HEXT IN
 * PH1 HEXT OUT
 */
  
#define LED0                    PC4
//#define LED1                    PD14

#define BEEPER                  PC15
#define BEEPER_INVERTED

// *************** Gyro & ACC **********************
#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7
#define SPI1_NSS_PIN            PA15

// #define USE_EXTI 
// #define GYRO_INT_EXTI           PA8
// #define USE_MPU_DATA_READY_SIGNAL

// MPU6500
#define USE_IMU_MPU6500
#define IMU_MPU6500_ALIGN       CW180_DEG 
#define MPU6500_SPI_BUS         BUS_SPI1
#define MPU6500_CS_PIN          SPI1_NSS_PIN

// ICM42605/ICM42688P
#define USE_IMU_ICM42605
#define IMU_ICM42605_ALIGN      CW180_DEG
#define ICM42605_SPI_BUS        BUS_SPI1
#define ICM42605_CS_PIN         SPI1_NSS_PIN
// #define ICM42605_EXTI_PIN       GYRO_INT_EXTI

// BMI270
#define USE_IMU_BMI270
#define IMU_BMI270_ALIGN        CW180_DEG
#define BMI270_SPI_BUS          BUS_SPI1
#define BMI270_CS_PIN           SPI1_NSS_PIN

// LSM6DXX
#define USE_IMU_LSM6DXX
#define IMU_LSM6DXX_ALIGN        CW180_DEG
#define LSM6DXX_CS_PIN           SPI1_NSS_PIN
#define LSM6DXX_SPI_BUS          BUS_SPI1


// *************** I2C/Baro/Mag/EXT*********************
#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C2_SCL                PB10        // SCL pad
#define I2C2_SDA                PB11        // SDA pad
#define USE_I2C_PULLUP

#define USE_BARO
#define BARO_I2C_BUS            BUS_I2C2
#define USE_BARO_BMP280
#define USE_BARO_DPS310

#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C2
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define DEFAULT_I2C_BUS         BUS_I2C2

// todo temp sensors
//#define TEMPERATURE_I2C_BUS     BUS_I2C1
// air speed sensors
//#define PITOT_I2C_BUS           BUS_I2C1
// 测距仪
//#define USE_RANGEFINDER
//#define RANGEFINDER_I2C_BUS         BUS_I2C1
 
// *************** OSD *****************************
#define USE_SPI_DEVICE_2 
#define SPI2_SCK_PIN            PB13//PB13 on LQFP64
#define SPI2_MISO_PIN           PB14//PB14 on LQFP64
#define SPI2_MOSI_PIN           PB15//PB15 on LQFP64
#define SPI2_NSS_PIN            PB12 //confirm on lqfp64

#define USE_MAX7456
#define MAX7456_SPI_BUS         BUS_SPI2
#define MAX7456_CS_PIN          SPI2_NSS_PIN


// *************** SD/BLACKBOX **************************
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN   	    PB4
#define SPI3_MOSI_PIN   	    PB5
#define SPI3_NSS_PIN 			PB9 //confirm on lqfp64
  
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define M25P16_SPI_BUS          BUS_SPI3
#define M25P16_CS_PIN           SPI3_NSS_PIN

#define USE_FLASH_W25N01G
#define W25N01G_SPI_BUS         BUS_SPI3
#define W25N01G_CS_PIN          SPI3_NSS_PIN

// *************** UART *****************************
#define USE_VCP
//#define USB_DETECT_PIN          PC14
#define USE_USB_DETECT

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART4
#define UART4_RX_PIN            PC11
#define UART4_TX_PIN            PC10

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define SERIAL_PORT_COUNT       6

#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_CRSF
#define SERIALRX_UART           SERIAL_PORT_USART2

// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE                ADC1
// todo
#define ADC1_DMA_STREAM             DMA2_CHANNEL5
#define ADC_CHANNEL_1_PIN           PC2
#define ADC_CHANNEL_2_PIN           PC1
//#define ADC_CHANNEL_3_PIN           PB0
#define VBAT_ADC_CHANNEL            ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL   ADC_CHN_2
//#define RSSI_ADC_CHANNEL            ADC_CHN_3 

#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_CURRENT_METER | FEATURE_TELEMETRY| FEATURE_VBAT | FEATURE_OSD )

// #define USE_LED_STRIP
// #define WS2811_PIN                      PB10   //TIM2_CH3

// // 遥测
// #define USE_SPEKTRUM_BIND
// #define BIND_PIN                   PA3    //UART2_RX_PIN

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff
#define TARGET_IO_PORTE         BIT(2)

#define MAX_PWM_OUTPUT_PORTS        8
#define USE_DSHOT
#define USE_ESC_SENSOR