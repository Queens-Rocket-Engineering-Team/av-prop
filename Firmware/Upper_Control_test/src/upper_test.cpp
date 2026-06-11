/*
 * Upper Prop. Control Module Testing FW
 * Authors: Jeevan Sanchez, Tristan Alderson
 * Hardware: Upper LC Board (PEGASUS)
 * Env: PlatformIO (ESP32)
 * Created: Apr.19.2026
 * Updated: May.7.2026
 * Purpose: SRAD firmware for peripheral testing of upper control module.
 *
 * Note: Hall sensor and buzzer have NOT been tested yet.
 *
 * QRET Avionics 2025-2026
 */

#include "pinouts.h"
#include <prop_testing.h>
#include <WiFi.h>
#include <FastLED.h>
#include <TMAG5273.h>

const int ledArray[] = {CAN_LED_PIN, WIFI_LED_PIN, DEBUG_LED_PIN};

ADS131M04 adc(-1, ADC_DRDY_PIN, &SPI); // -1 for no CS pin (tied low)

TMAG5273 hallSensor;

#define NUM_LEDS 1
CRGB rgb_leds[NUM_LEDS];

// WIFI credentials (replace)
const char *ssid = "";
const char *pswd = "";

// WIFI testing
void wifiTest(const char *ssid, const char *pswd)
{
    Serial.printf("\nconnecting to: %s\n", ssid);
    WiFi.begin(ssid, pswd);

    int timeout_idx = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        blinkLed(WIFI_LED_PIN, 300);
        Serial.print(".");
        timeout_idx++;
        if (timeout_idx == 10)
            break;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.printf("\nconnected to: %s\n", ssid);
        digitalWrite(WIFI_LED_PIN, HIGH);
    }
    else
    {
        Serial.println("\nFailed to connect.");
    }
}

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

            Serial.print("PT1: ");
            Serial.print(psi1, 1);
            Serial.print(" PSI | V_PT: ");
            Serial.print(volts[0], 4);

            Serial.print(" | PT2: ");
            Serial.print(psi2, 1);
            Serial.print(" PSI | V_PT: ");
            Serial.print(volts[1], 4);
        }
    }
}

bool hallPolling = false;
unsigned long lastHallPoll = 0;
const int hallPollInterval = 250;

void pollHall()
{
    float axes[3];

    // obtain and log magnetic flux data
    if (hallSensor.getAllFlux(axes))
    {
        Serial.print("X: ");
        Serial.print(axes[0], 2);

        Serial.print(" | Y: ");
        Serial.print(axes[1], 2);

        Serial.print(" | Z: ");
        Serial.print(axes[2], 2);

        Serial.println(" mT");
    }

    // obtain and log temperature data
    float temp = hallSensor.getTemp();
    if (!isnan(temp))
    {
        Serial.print(" | Temp: ");
        Serial.print(temp, 2);
        Serial.println(" C");
    }
}

void setup()
{
    Serial.begin(38400);

    // ADC clock setup
    ledcSetup(0, 8192000, 1); // 8.192MHz, 1-bit resolution
    ledcAttachPin(ADC_CLKIN_PIN, 1);
    ledcWrite(0, 1);

    SPI.begin(ADC_SCLK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, -1);
    Wire.begin(HALL_SDA_PIN, HALL_SCL_PIN);

    adcSetup(adc);

    if (!hallSensor.init(0x35, Wire))
    {
        Serial.println("TMAG5273 init failed");
        while (1)
            delay(100);
    }

    pinMode(VPT_EN_PIN, OUTPUT);
    pinMode(SOL1_EN_PIN, OUTPUT);
    pinMode(SOL2_EN_PIN, OUTPUT);
    pinMode(BUZZ_EN_PIN, OUTPUT);

    // setup LEDC (osc) for buzzer
    // frequency: 4000Hz, resolution: 8 bits
    ledcSetup(1, 4000, 8);
    ledcAttachPin(BUZZ_EN_PIN, 1);
    ledcWrite(1, 0); // silent start

    for (int i = 0; i < 3; i++)
    {
        pinMode(ledArray[i], OUTPUT);
        digitalWrite(ledArray[i], LOW);
    }

    digitalWrite(VPT_EN_PIN, LOW);
    digitalWrite(SOL1_EN_PIN, LOW);
    digitalWrite(SOL2_EN_PIN, LOW);

    analogReadResolution(12);

    // RGB LED setup https://fastled.io/docs/
    FastLED.addLeds<WS2812B, RGB_DATA_PIN, GRB>(rgb_leds, NUM_LEDS);
    rgb_leds[0] = CRGB::Black;
    FastLED.show();

    Serial.println("start PEGASUS upper control module testing");
    Serial.println("1: solenoid 1 | 2: solenoid 2 | 3: analog sensing results | 4: hall sensor results | 5: 24V sense | 6: LEDs | 7: WiFi | 8: Buzzer");
}

void loop()
{
    uint8_t fluidBrightness = beatsin8(35, 20, 90);
    rgb_leds[0] = CHSV(55, 255, fluidBrightness);
    FastLED.show();

    pollADC();

    if (hallPolling && (millis() - lastHallPoll >= hallPollInterval))
    {
        lastHallPoll = millis();
        pollHall();
    }


    if (Serial.available() > 0)
    {
        char input = Serial.read();

        switch (input)
        {
        case '1':
            enableValve(SOL1_EN_PIN);
            delay(5000);
            disableValve(SOL1_EN_PIN);
            break;

        case '2':
            enableValve(SOL2_EN_PIN, 2);
            delay(5000);
            disableValve(SOL2_EN_PIN);
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
            break;

        case '5':
        {
            uint32_t mv = analogReadMilliVolts(VSOL_SENSE_PIN);
            float vPin = mv / 1000.0f;
            float vActual = vPin * 11.0f;

            Serial.printf("24v Sense: %5.2f V | Raw mV: %u\n", vActual, mv);
            break;
        }

        case '6':
            flashLeds(ledArray, 3);
            break;

        case '7':
            wifiTest(ssid, pswd);
            break;

        case '8':
            ledcWrite(1, 128);
            delay(1000);
            ledcWrite(1, 0);
            break;

        default:
            break;
        }
    }
}
