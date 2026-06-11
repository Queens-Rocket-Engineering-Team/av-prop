/*
 * Lower Prop. Control Module Testing FW
 * Authors: Jeevan Sanchez, Tristan Alderson
 * Hardware: Lower LC Board (HYDRA)
 * Env: PlatformIO (STM32)
 * Created: Apr.19.2026
 * Updated: May.7.2026
 * Purpose: SRAD firmware for peripheral testing of lower control module.
 *
 * Note: Hall sensors have NOT been tested yet.
 *
 * QRET Avionics 2025-2026
 */

#include "pinouts.h"
#include <SoftwareSerial.h>
#include <ADS131M04.h>
#include <TMAG5273.h>
#include <SPIFlash.h>
#include <FastLED.h>
#include <prop_testing.h>
#include <AimFileSystem.h>
#include <AimFlightRecorder.h>
#include <AimConfigStore.h>

SoftwareSerial serial(USART_RX_PIN, USART_TX_PIN);

const int ledArray[] = {CAN_LED_PIN, DB_LED_PIN};

ADS131M04 adc(ADC_CS_PIN, ADC_DRDY_PIN, &SPI);

struct LowerConfig {
    // Application configuration
};

static LowerConfig g_appConfig;
static SerialFlashDriver g_flashHw(FL_CS_PIN);
static AimFileSystem g_fs(&g_flashHw);
static AimFlightRecorder g_flightRecorder(g_fs, 8, 100, 2 * 1024 * 1024);
static AimConfigStore g_configStore(g_fs);

#define NUM_LEDS 1
CRGB rgb_leds[NUM_LEDS];

TwoWire Wire1(HALL_SDA1_PIN, HALL_SCL1_PIN);
TwoWire Wire2(HALL_SDA2_PIN, HALL_SCL2_PIN);

TMAG5273 hallSensor1;
TMAG5273 hallSensor2;

bool isADCPolling = false;
unsigned long lastADCPollTime = 0;
const int ADCPollInterval = 500;

// helper for polling ADC reads
void pollADC()
{
    if (isADCPolling && (millis() - lastADCPollTime >= ADCPollInterval))
    {
        lastADCPollTime = millis();
        int32_t rawData[4];
        float volts[4];

        // readChannels method is true on a successful ADC read
        if (adc.readChannels(rawData))
        {
            adc.computeVoltages(rawData, volts);

            float psi1 = processPT(volts[0]);
            float psi2 = processPT(volts[1]);

            float tempC = processTC(volts[2], CJC_SENSE_PIN);
            float coldJunc = readColdJunction(CJC_SENSE_PIN);

            serial.print("PT1: ");
            serial.print(psi1, 1);
            serial.print(" PSI | V_PT: ");
            serial.print(volts[0], 4);

            serial.print("PT2: ");
            serial.print(psi2, 1);
            serial.print(" PSI | V_PT: ");
            serial.print(volts[1], 4);

            serial.print(" | TC: ");
            serial.print(tempC, 1);
            serial.print(" C | CJC: ");
            serial.print(coldJunc, 1);

            serial.print(" | V_TC: ");
            serial.println(volts[2], 6);
        }
    }
}

bool hallPolling = false;
unsigned long lastHallPoll = 0;
const int hallPollInterval = 250;

void pollHall()
{
    float axes1[3];
    float axes2[3];

    if (hallSensor1.getAllFlux(axes1))
    {
        serial.print("H1 X: ");
        serial.print(axes1[0], 2);

        serial.print(" | Y: ");
        serial.print(axes1[1], 2);

        serial.print(" | Z: ");
        serial.print(axes1[2], 2);
    }

    if (hallSensor2.getAllFlux(axes2))
    {
        serial.print(" || H2 X: ");
        serial.print(axes2[0], 2);

        serial.print(" | Y: ");
        serial.print(axes2[1], 2);

        serial.print(" | Z: ");
        serial.print(axes2[2], 2);
    }

    float temp1 = hallSensor1.getTemp();
    float temp2 = hallSensor2.getTemp();

    serial.print(" | T1: ");
    serial.print(temp1, 1);

    serial.print(" C | T2: ");
    serial.print(temp2, 1);
    serial.println(" C");
}

void setup()
{
    serial.begin(9600);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    HardwareTimer *adcCk = new HardwareTimer(TIM2);
    adcCk->setPWM(1, PA0, 8192000, 50);
    adcCk->resume();

    SPI.begin();
    Wire1.begin();
    Wire2.begin();

    if (g_fs.begin()) {
        // Storage initialized
    }

    adc.init();

    if (!hallSensor1.init(0x35, Wire1))
    {
        Serial.println("TMAG5273 init failed");
        while (1)
            delay(100);
    }

    if (!hallSensor2.init(0x35, Wire2))
    {
        Serial.println("TMAG5273 init failed");
        while (1)
            delay(100);
    }

    pinMode(VPT_EN_PIN, OUTPUT);
    pinMode(VSOL_EN_PIN, OUTPUT);
    pinMode(SOL1_EN_PIN, OUTPUT);
    pinMode(SOL2_EN_PIN, OUTPUT);

    disablePower(VPT_EN_PIN);
    disablePower(VSOL_EN_PIN);

    for (int i = 0; i < 2; i++)
    {
        pinMode(ledArray[i], OUTPUT);
        digitalWrite(ledArray[i], LOW);
    }

    analogReadResolution(12);

    FastLED.addLeds<WS2812B, RGB_DATA_PIN, GRB>(rgb_leds, NUM_LEDS);
    rgb_leds[0] = CRGB::Black;
    FastLED.show();

    Serial.println("start HYDRA lower control module testing");
    Serial.println("1: solenoid 1 | 2: solenoid 2 | 3: analog sensing results | 4: hall sensor results | 5: 24V sense | 6: LEDs");
}

void loop()
{
    uint8_t fluidBrightness = beatsin8(35, 20, 90);
    rgb_leds[0] = CHSV(135, 255, fluidBrightness);
    FastLED.show();

    pollADC();

    if (hallPolling && (millis() - lastHallPoll >= hallPollInterval))
    {
        lastHallPoll = millis();
        pollHall();
    }

    if (serial.available() > 0)
    {
        char input = serial.read();
        switch (input)
        {
        case '1':
            enablePower(VSOL_EN_PIN);
            enableValve(SOL1_EN_PIN);
            delay(5000);
            disableValve(SOL1_EN_PIN);
            disablePower(VSOL_EN_PIN);
            break;

        case '2':
            enablePower(VSOL_EN_PIN);
            enableValve(SOL2_EN_PIN);
            delay(5000);
            disableValve(SOL2_EN_PIN);
            disablePower(VSOL_EN_PIN);
            break;

        case '3':
            isADCPolling = !isADCPolling;
            if (isADCPolling)
            {
                enablePower(VPT_EN_PIN);
            }
            else
            {
                disablePower(VPT_EN_PIN);
            }
            break;

        case '4':
            hallPolling = !hallPolling;

            if (hallPolling)
            {
                lastHallPoll = millis();
                pollHall();
            }
            break;

        case '5':
        {
            uint32_t rawVSOL = analogRead(VSOL_SENSE_PIN);
            float vPin = (rawVSOL / 4095.0f) * 3.3f;
            float vActual = vPin * 11.0f;

            Serial.printf("VSOL: %5.2f V | Raw: %u\n", vActual, rawVSOL);
            break;
        }

        case '6':
            flashLeds(ledArray, 2);
            break;
        }
    }
}