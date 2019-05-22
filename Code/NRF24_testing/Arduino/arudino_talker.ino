#include "payload.h"
 
#define SERIAL_DEBUG true
#include <SerialDebug.h>
 
#define LED_DEBUG true
#include <LedDebug.h>
 
#define SENSE_DELAY 2000
Payload payload = (Payload) { METEO };
 
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
 
uint32_t lastSense;
inline bool sense() {
    long now = millis();
    if (now < lastSense || now - lastSense > SENSE_DELAY) {
        payload.data.meteo.humidity = millis();
        payload.data.meteo.temperature = millis() * 50;
        payload.data.meteo.pressure = millis();
        payload.data.meteo.altitude = millis();
        payload.data.meteo.luminosity = map(analogRead(A0), 0, 1024, 100, 0);
        lastSense = now;
        return true;
    } else {
        return false;
    }
}
 
#define RADIO_CE_PIN 9
#define RADIO_CS_PIN 10
#define RADIO_RETRY_DELAY 15
#define RADIO_RETRY_COUNT 15
RF24 radio = RF24(RADIO_CE_PIN, RADIO_CS_PIN);
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };
 
void setup() {
    SERIAL_DEBUG_SETUP(9600);
    pinMode(A0, INPUT);
 
    radio.begin();
    radio.setRetries(RADIO_RETRY_DELAY, RADIO_RETRY_COUNT);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_MAX);
    radio.setPayloadSize(sizeof(payload));
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);
}
 
void loop() {
    if (sense()) {
        radio.powerUp();
        if (!radio.write(&payload, sizeof(payload))) {
            PULSE(3,75);
        } else {
            PULSE(1,225);
        }
        radio.powerDown();
 
        DEBUG("payload", sizeof(payload), "enum", sizeof(payload.type));
    }
}
