/**
 * @file http-server_main.cpp
 * @brief Example using EZModbus for a Modbus JSON HTTP Server application
 */

/**
 * Test of the ModbusHTTP server
 * -----------------------------
 * 
 * This file implements a minimal, dummy Modbus/HTTP server with :
 * - 1 holding register (HR0) : Temperature (°C x10) - read/write
 * - 1 input register (IR0) : Humidity (% x10) - read only
 * - 1 coil (CO0) : Relay - read/write
 * - 1 discrete input (DI0) : Motion sensor - read only
 * 
 * Tests with CURL :
 * ---------------
 * 1. Read temperature (HR0) :
 *    curl -X POST http://<IP_ESP32>/modbus -H "Content-Type: application/json" \
 *    -d '{"type":"request","slaveId":1,"fc":3,"regAddress":0,"regCount":1}'
 * 
 * 2. Write temperature (HR0) :
 *    curl -X POST http://<IP_ESP32>/modbus -H "Content-Type: application/json" \
 *    -d '{"type":"request","slaveId":1,"fc":6,"regAddress":0,"regCount":1,"data":[300]}'
 * 
 * 3. Read humidity (IR0) :
 *    curl -X POST http://<IP_ESP32>/modbus -H "Content-Type: application/json" \
 *    -d '{"type":"request","slaveId":1,"fc":4,"regAddress":0,"regCount":1}'
 * 
 * 4. Read relay (CO0) :
 *    curl -X POST http://<IP_ESP32>/modbus -H "Content-Type: application/json" \
 *    -d '{"type":"request","slaveId":1,"fc":1,"regAddress":0,"regCount":1}'
 * 
 * 5. Write relay (CO0) :
 *    curl -X POST http://<IP_ESP32>/modbus -H "Content-Type: application/json" \
 *    -d '{"type":"request","slaveId":1,"fc":5,"regAddress":0,"regCount":1,"data":[1]}'
 * 
 * 6. Read motion sensor (DI0) :
 *    curl -X POST http://<IP_ESP32>/modbus -H "Content-Type: application/json" \
 *    -d '{"type":"request","slaveId":1,"fc":2,"regAddress":0,"regCount":1}'
 * 
 * Error tests :
 * --------------
 * 1. Bad slave ID (2) :
 *    curl -X POST http://<IP_ESP32>/modbus -H "Content-Type: application/json" \
 *    -d '{"type":"request","slaveId":2,"fc":3,"regAddress":0,"regCount":1}'
 *    (should be ignored)
 * 
 * 2. Bad register address (HR1) :
 *    curl -X POST http://<IP_ESP32>/modbus -H "Content-Type: application/json" \
 *    -d '{"type":"request","slaveId":1,"fc":3,"regAddress":1,"regCount":1}'
 *    (should be ignored or return an exception depending on ModbusServer configuration)
 * 
 * 3. Write out of range (>100°C) :
 *    curl -X POST http://<IP_ESP32>/modbus -H "Content-Type: application/json" \
 *    -d '{"type":"request","slaveId":1,"fc":6,"regAddress":0,"regCount":1,"data":[1001]}'
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <EZModbus.h>
#include "ModbusJsonCodec.h"
#include "ModbusHTTP.h"

// Wi-Fi credentials
const char* ssid = "your_ssid_here";
const char* password = "your_password_here";

// Aliases for convenience
using ModbusServer = Modbus::Server;
using ModbusRegister = ModbusServer::Register;

// HTTP server instance
ModbusHTTP modbusHTTP(80);

// Modbus server instance
// Slave ID = 1, reject undefined registers
ModbusServer modbusServer(modbusHTTP, 1, true);

// Variables to simulate data
uint16_t temperature = 250;  // 25.0°C
uint16_t humidity = 650;     // 65.0%
bool relayState = false;     // Relay state
bool motionDetected = false; // Motion sensor state


void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting ModbusHTTP server...");

    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("WiFi connected, IP : ");
    Serial.println(WiFi.localIP());

    // Define Modbus registers
    
    // Holding register for temperature (read/write)
    ModbusRegister tempReg = {
        .type = ModbusServer::HOLDING_REGISTER,
        .address = 0,
        .readCb = [](const ModbusRegister& ctx) -> uint16_t {
            Serial.printf("Read temperature: %.1f°C\n", temperature/10.0f);
            return temperature;
        },
        .writeCb = [](uint16_t value, const ModbusRegister& ctx) -> bool {
            if (value > 1000) return false; // Max 100.0°C
            temperature = value;
            Serial.printf("Write temperature: %.1f°C\n", temperature/10.0f);
            return true;
        },
        .name = "Temperature"
    };
    modbusServer.addRegister(tempReg);

    // Input register for humidity (read only)
    ModbusRegister humReg = {
        .type = ModbusServer::INPUT_REGISTER,
        .address = 0,
        .readCb = [](const ModbusRegister& ctx) -> uint16_t {
            Serial.printf("Read humidity: %.1f%%\n", humidity/10.0f);
            return humidity;
        },
        .name = "Humidity"
    };
    modbusServer.addRegister(humReg);

    // Coil for relay (read/write)
    ModbusRegister relayReg = {
        .type = ModbusServer::COIL,
        .address = 0,
        .readCb = [](const ModbusRegister& ctx) -> uint16_t {
            Serial.printf("Read relay: %s\n", relayState ? "ON" : "OFF");
            return relayState ? 1 : 0;
        },
        .writeCb = [](uint16_t value, const ModbusRegister& ctx) -> bool {
            relayState = value != 0;
            Serial.printf("Write relay: %s\n", relayState ? "ON" : "OFF");
            return true;
        },
        .name = "Relay"
    };
    modbusServer.addRegister(relayReg);

    // Discrete Input for motion sensor (read only)
    ModbusRegister motionReg = {
        .type = ModbusServer::DISCRETE_INPUT,
        .address = 0,
        .readCb = [](const ModbusRegister& ctx) -> uint16_t {
            Serial.printf("Read motion sensor: %s\n", motionDetected ? "TRIGGERED" : "NONE");
            return motionDetected ? 1 : 0;
        },
        .name = "Motion"
    };
    modbusServer.addRegister(motionReg);

    // Start HTTP server
    modbusHTTP.begin();
    Serial.println("ModbusHTTP server started");
    Serial.println("Registers configured:");
    Serial.println("- Holding Register 0: Temperature (°C x10)");
    Serial.println("- Input Register 0:   Humidity (% x10)");
    Serial.println("- Coil 0:             Relay (ON/OFF)");
    Serial.println("- Discrete Input 0:   Motion sensor (ON/OFF)");
}

void loop() {
    // Handle Modbus requests
    modbusServer.poll();
    
    // Simulate data variation
    static uint32_t lastUpdate = 0;
    if (millis() - lastUpdate >= 5000) {
        lastUpdate = millis();
        
        // Random temperature variation ±0.5°C
        temperature += random(-5, 6);
        if (temperature > 500) temperature = 500;  // Max 50.0°C
        if (temperature < 100) temperature = 100;  // Min 10.0°C
        
        // Random humidity variation ±1%
        humidity += random(-10, 11);
        if (humidity > 1000) humidity = 1000;  // Max 100.0%
        if (humidity < 0) humidity = 0;        // Min 0%

        // Random motion sensor state change
        if (random(100) < 20) { // 20% chance of changing state
            motionDetected = !motionDetected;
            Serial.printf("Motion sensor state change: %s\n", 
                         motionDetected ? "TRIGGERED" : "NONE");
        }
    }
} 