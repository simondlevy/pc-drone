/*
 * Uses a Teensy board and four MCP4725 Digital-Analog Converters to send stick
 * demands to a Cheerson CX-10 transmitter
 *
 * Copyright (c) 2023 Simon D. Levy
 *
 * MIT License
 */

#include <Wire.h>
#include <Adafruit_MCP4725.h>

static const uint8_t POWER_PIN = 10;

static const bool DEBUG = false;

// Shorthand
typedef Adafruit_MCP4725 Dac;

// DACs
static Dac dacT;  // throttle
static Dac dacR;  // roll
static Dac dacP;  // pitch
static Dac dacY;  // yaw

static void writeDemand(Dac & dac, const uint16_t pwm)
{
    dac.setVoltage(4095 * (pwm - 1000) / 1000.,
            false); // false = don't write EEPROM
}

static void writeDemandInverse(Dac & dac, const uint16_t pwm)
{
    writeDemand(dac, 3000 - pwm);
}

static void writeThrottle(const uint16_t t)
{
    writeDemandInverse(dacT, t);
}

static void writeDemands(
        const uint16_t t,
        const uint16_t r,
        const uint16_t p,
        const uint16_t y)
{
    writeThrottle(t);

    writeDemandInverse(dacR, r);
    writeDemand(dacP, p);
    writeDemandInverse(dacY, y);
}

void setup(void) 
{
    // Start serial comms for demand input
    Serial.begin(115200);

    // Start serial comms for debugging if indicated
    if (DEBUG) {
        Serial1.begin(115200);
    }

    // Start DACs
    dacT.begin(0x60, &Wire);
    dacR.begin(0x61, &Wire);
    dacP.begin(0x60, &Wire1);
    dacY.begin(0x61, &Wire1);

    // Turn off the transmitter
    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, LOW);

    // Turn off LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Poll for input connection
    while (!Serial.available()) {
    }

    // Turn the transmitter on
    digitalWrite(POWER_PIN, HIGH);

    // Turn LED on
    digitalWrite(LED_BUILTIN, HIGH);

    // Wait a second
    delay(1000);

    // Throttle up and down to arm
    writeThrottle(2000);  
    delay(1000);
    writeThrottle(1000);  
}

static void getDemands(
        const uint8_t curr_byte,
        uint16_t & throttle,
        uint16_t & roll,
        uint16_t & pitch,
        uint16_t & yaw)
{
    static uint8_t index;
    static uint8_t buff[8];

    buff[index++] = curr_byte;

    // When index reaches 10, we have two sentinel bytes and an
    // eight-byte demands message
    if (index == 10) {

        uint16_t demands[4];
        memcpy(demands, &buff[2], 8); // skip sentinel bytes
        index = 0;

        throttle = demands[0];
        roll = demands[1];
        pitch = demands[2];
        yaw = demands[3];

        if (DEBUG) {
            Serial1.printf("t=%d  r=%d  p=%d  y=%d\n", 
                    throttle, roll, pitch, yaw);
        }
    }
}

static void handleInputByte(
        const uint8_t curr_byte,
        uint16_t & throttle,
        uint16_t & roll,
        uint16_t & pitch,
        uint16_t & yaw)
{
    static bool ready;
    static uint8_t prev_byte;

    // We're ready to read messages when we get two zero bytes in a row
    if (curr_byte == 0 and prev_byte == 0) {
        ready = true;
    }

    if (ready) {

        getDemands(curr_byte, throttle, roll, pitch, yaw);

    }

    prev_byte = curr_byte;
}

void loop(void) 
{
    static uint16_t throttle, roll, pitch, yaw;
    static bool throttle_ready;
    static uint32_t msec_prev;

    while (Serial.available()) {

        handleInputByte(Serial.read(), throttle, roll, pitch, yaw);

        msec_prev = millis();
    }

    // Failsafe: cut throttle on input disconnect
    if (millis() - msec_prev > 1000) {
        digitalWrite(LED_BUILTIN, LOW);
        throttle = 1000;
    }

    if (throttle == 1000) {
        throttle_ready = true;
    }

    if (throttle_ready) {
        writeDemands(throttle, roll, pitch, yaw);
    }

}
