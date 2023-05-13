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

static Adafruit_MCP4725 dacT;   // throttle
static Adafruit_MCP4725 dacR;   // roll
static Adafruit_MCP4725 dacP;   // pitch
static Adafruit_MCP4725 dacY;   // yaw

static void writeThrottle(const uint16_t u)
{
    const uint16_t v = 4095 - 4095 * (u - 1000) / 1000.;

    dacT.setVoltage(v, false); // false = don't write EEPROM
}

void setup(void) 
{
    // Start serial comms for demand input
    Serial.begin(115200);

    // Start serial comms for debugging
    Serial1.begin(115200);

    dacT.begin(0x60, &Wire);
    dacR.begin(0x61, &Wire);
    dacP.begin(0x60, &Wire1);
    dacY.begin(0x61, &Wire1);

    // Turn off the transmitter
    pinMode(POWER_PIN, OUTPUT);
    /*
    digitalWrite(POWER_PIN, LOW);

    // Poll for Enter
    while (true) {
        Serial.println("Hit Enter to begin...");
        if (Serial.available()) {
            break;
        }
        delay(1000);
    }*/

    // Turn the transmitter on
    digitalWrite(POWER_PIN, HIGH);

    /*
    // Wait a couple of seconds
    delay(2000);

    Serial.println("Here we go!");

    // Throttle up and down to arm
    writeThrottle(2000);  
    delay(1000);
    writeThrottle(1000);  

    // Wait a couple more seconds
    delay(2000);

    // Throttle up bit to spin the motors
    writeThrottle(1200);  

    // Run for a few seconds
    delay(5000);

    // Throttle back down
    writeThrottle(1000); 
    */
}

void loop(void) 
{
    /*
    while (Serial.available()) {
    }*/

    Serial1.println("hello");
}
