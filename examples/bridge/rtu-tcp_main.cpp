/**
 * @file rtu-tcp_main.cpp
 * @brief Example using EZModbus for a Modbus Bridge application
 * @brief (demonstration of bridging between RTU Client and TCP Server)
 */

#include <Arduino.h>
#include <WiFi.h>
#include "EZModbus.h"

// Configuration WiFi
#define WIFI_SSID "My-WiFi-Network"  // Your SSID here
#define WIFI_PASS "mypassword1234"   // Your password here
#define MODBUS_TCP_PORT 502        // Port Modbus TCP standard

// Configuration RTU
#define RS485_SERIAL Serial2
#define RS485_BAUD_RATE 9600
#define RS485_CONFIG SERIAL_8N1
#define RS485_RX_PIN 16
#define RS485_TX_PIN 17
#define RS485_DE_PIN 5  // DE/RE pin for RS485 communication

// Aliases for convenience
using UART = ModbusHAL::UART;
using TCP = ModbusHAL::TCP;
using ModbusRTU = ModbusInterface::RTU;
using ModbusTCP = ModbusInterface::TCP;
using ModbusBridge = Modbus::Bridge;

// UART port for Modbus RTU & TCP server for Modbus TCP
UART uart(RS485_SERIAL, RS485_BAUD_RATE, RS485_CONFIG, RS485_RX_PIN, RS485_TX_PIN, RS485_DE_PIN);
TCP tcpServer(MODBUS_TCP_PORT);

// Modbus interfaces
ModbusRTU rtuInterface(uart, Modbus::CLIENT);
ModbusTCP tcpInterface(tcpServer, Modbus::SERVER);

// Modbus bridge
ModbusBridge bridge(rtuInterface, tcpInterface);

void setup() {
    // Initialize serial for debug output
    Serial.begin(115200);
    Serial.println("\nEZModbus Bridge Example");
    
    // Initialize WiFi in Station mode
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    
    Serial.print("Connected to WiFi, IP address: ");
    Serial.println(WiFi.localIP());
    
    // Start UART & TCP server
    uart.begin();
    tcpServer.begin();
    Serial.println("UART & TCP Server started");
    
    // The bridge takes care of initializing the interfaces
    if (bridge.begin() != ModbusBridge::SUCCESS) {
        Serial.println("Failed to initialize Modbus bridge");
        while (1) { delay(1000); } // Halt
    }
    
    Serial.println("Modbus bridge initialized");
}

void loop() {
    // Nothing to do here - the bridge is polled in the background
    vTaskDelete(NULL);
}
