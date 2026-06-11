// Header file containing pinouts for this module 

#ifndef PINOUT_H
#define PINOUT_H

#define FL_CS_PIN PB0
#define VSOL_SENSE_PIN PB1
#define CAN_LED_PIN PB4
#define RGB_DATA_PIN PB5

// I2C-1 BUS
#define HALL_SCL1_PIN PB6
#define HALL_SDA1_PIN PB7

#define CAN_RX_PIN PB8
#define CAN_TX_PIN PB9

// I2C-2 BUS
#define HALL_SCL2_PIN PB10
#define HALL_SDA2_PIN PB11

#define RESET_FL_PIN PB12
#define SOL1_EN_PIN PB13
#define SOL2_EN_PIN PB14

#define ADC_CLKIN_PIN PA0
#define ADC_DRDY_PIN PA1
#define VPT_EN_PIN PA2
#define ADC_CS_PIN PA3
#define CJC_SENSE_PIN PA4 // adc12_in4

// SPI-1 BUS (ADC & FLASH MODULE)
#define SPI_SCLK_PIN PA5
#define SPI_MISO_PIN PA6
#define SPI_MOSI_PIN PA7

#define DB_LED_PIN PA8

// USART-1 (swapped cause of hardware issue)
#define USART_TX_PIN PA9
#define USART_RX_PIN PA10

#define VSOL_EN_PIN PA11

// SW DEBUG
#define SWDIO_PIN PA13
#define SWCLK_PIN PA14

#endif

