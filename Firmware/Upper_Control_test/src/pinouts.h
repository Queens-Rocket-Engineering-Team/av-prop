// Header file containing pinouts for this module 

#define CAN_TX_PIN 1 
#define CAN_RX_PIN 2 
#define SENSE_24V_PIN 7 
#define SOL1_EN_PIN 9
#define SOL2_EN_PIN 10

// ADC SPI
#define ADC_CLKIN_PIN 11
#define ADC_MOSI_PIN 12
#define ADC_MISO_PIN 13
#define ADC_SCLK_PIN 14

// HALL I2C 
#define HALL_SCL_PIN 17
#define HALL_SDA_PIN 18

#define ADC_DRDY_PIN 19

// iterable
const int WIFI_LED_PIN = 35;
const int CAN_LED_PIN = 36;
const int DEBUG_LED_PIN = 37;

#define BUZZ_EN_PIN 41
#define RGB_DATA_PIN 42
#define VPT_EN_PIN 47