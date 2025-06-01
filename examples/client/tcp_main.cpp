/**
 * @file tcp_main.cpp
 * @brief Example using EZModbus for a thermostat application
 * @brief (demonstration of Modbus TCP Client usage)
 */

#include <Arduino.h>
#include <WiFi.h>
#include "EZModbus.h"

// WiFi configuration
#define WIFI_SSID "My-WiFi-Network"  // Your SSID here
#define WIFI_PASS "mypassword1234" // Your password here

// Modbus TCP server configuration
#define MODBUS_IP "192.168.1.24" // Modbus TCP server IP address
#define MODBUS_PORT 502  // Port Modbus TCP standard

// Thermostat Modbus configuration
#define THERMOSTAT_SLAVE_ID 1

// Register map for our example thermostat
// Coils (read/write)
#define REG_TEMP_REGULATION_ENABLE 100      // Temperature regulation enable
#define REG_HUMIDITY_REGULATION_ENABLE 101  // Humidity regulation enable
    
// Discrete Inputs (read only)
#define REG_ALARM_START 200                 // 10 discrete inputs for alarms (200-209)
    
// Input Registers (read only)
#define REG_CURRENT_TEMPERATURE 300         // Current temperature (°C × 10)
#define REG_CURRENT_HUMIDITY 301            // Current humidity (% × 10)
    
// Holding Registers (read/write)
#define REG_TEMPERATURE_SETPOINT 400        // Temperature setpoint (°C × 10)
#define REG_HUMIDITY_SETPOINT 401           // Humidity setpoint (% × 10)

// Aliases for convenience
using TCP = ModbusHAL::TCP;
using ModbusTCP = ModbusInterface::TCP;
using ModbusClient = Modbus::Client;

// TCP client for Modbus TCP
TCP tcpClient(MODBUS_IP, MODBUS_PORT);

// Modbus TCP interface and client
ModbusTCP interface(tcpClient, Modbus::CLIENT);
ModbusClient client(interface);

// Function prototypes
void readTemperature_Sync();
void readAlarms_Async();
void readSetpoints_Sync();
void writeSetpoints_Async();

// ===================================================================================
// SETUP & LOOP
// ===================================================================================

void setup() {
    // Initialize serial for debug output
    Serial.begin(115200);
    Serial.println("\nEZModbus Thermostat Client Example");
    
    // Connect to WiFi
    Serial.print("Connecting to WiFi");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Initialize TCP driver
    tcpClient.begin();
    
    // Initialize Modbus TCP Client 
    if (client.begin() != ModbusClient::SUCCESS) {
        Serial.println("Failed to initialize Modbus Client");
        while (1) { delay(1000); } // Halt
    }
    
    Serial.println("Modbus client initialized");
}

void loop() {
    // Check WiFi connection
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWiFi connection lost! Reconnecting...");
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        Serial.println("\nWiFi reconnected");
    }
    
    // Execute each example in sequence with a delay between them
    Serial.println("\n========== Starting EZModbus Examples ==========");
    
    // Example 1: Synchronous read
    Serial.println("\n****** EXAMPLE 1: Synchronous Read ******");
    readTemperature_Sync();
    delay(3000);
    
    // Example 2: Asynchronous read
    Serial.println("\n****** EXAMPLE 2: Asynchronous Read ******");
    readAlarms_Async();
    delay(3000);
    
    // Example 3: Synchronous read using raw frame
    Serial.println("\n****** EXAMPLE 3: Raw Frame Read ******");
    readSetpoints_Sync();
    delay(3000);
    
    // Example 4: Asynchronous write
    Serial.println("\n****** EXAMPLE 4: Asynchronous Write ******");
    writeSetpoints_Async();
    delay(3000);
    
    Serial.println("\n========== All Examples Completed ==========");
    Serial.println("Waiting 10 seconds before running again...");
    delay(10000);
}

// ===================================================================================
// EXAMPLE CASES
// ===================================================================================

/**
 * Example 1: Synchronous read of current temperature and humidity
 */
void readTemperature_Sync() {
    Serial.println("Reading current temperature and humidity...");
    
    // Create frame to read temperature (input register)
    Modbus::Frame tempRequest = {
        .type = Modbus::REQUEST,
        .fc = Modbus::READ_INPUT_REGISTERS,
        .slaveId = THERMOSTAT_SLAVE_ID,
        .regAddress = REG_CURRENT_TEMPERATURE,
        .regCount = 1,
        .data = {}
    };
    
    // Send request and wait for response
    // (tracker not provided -> blocks until response received or timeout)
    Modbus::Frame tempResponse;
    auto result = client.sendRequest(tempRequest, tempResponse);
    
    // Check if the request was successful
    if (result != ModbusClient::SUCCESS) {
        Serial.printf("Failed to read temperature: %s\n", ModbusClient::toString(result));
        return;
    }

    // Check if the response has an exception
    if (tempResponse.exceptionCode != Modbus::NULL_EXCEPTION) {
        Serial.printf("Modbus exception reading temperature: %s\n", Modbus::toString(tempResponse.exceptionCode));
        return;
    }

    // Get the temperature value from the response
    float tempValue = tempResponse.getRegister(0) / 10.0f;
    Serial.printf("Temperature: %.1f°C\n", tempValue);
}

/**
 * Example 2: Asynchronous read of alarm status
 */
void readAlarms_Async() {
    Serial.println("Reading alarm status...");
    
    // Create frame to read multiple discrete inputs
    Modbus::Frame request = {
        .type = Modbus::REQUEST,
        .fc = Modbus::READ_DISCRETE_INPUTS,
        .slaveId = THERMOSTAT_SLAVE_ID,
        .regAddress = REG_ALARM_START,
        .regCount = 10,  // Read 10 alarms
        .data = {}
    };
    
    // Create frame for response and status tracker
    Modbus::Frame response;
    ModbusClient::Result tracker;
    
    // Send request asynchronously
    // (tracker provided -> returns immediately after transfer started)
    auto result = client.sendRequest(request, response, &tracker);
    
    if (result != ModbusClient::SUCCESS) {
        Serial.printf("Failed to start alarm read: %s\n", ModbusClient::toString(result));
        return;
    }
    
    Serial.println("Alarm read request sent. Waiting for completion...");
    
    // Wait for the request to complete
    // (usually this is done in another task/function)
    uint32_t startTime = millis();
    while (tracker == ModbusClient::NODATA) {
        // Timeout after 2 seconds (safety, should be already handled by the ModbusClient)
        if (millis() - startTime > 2000) {
            Serial.println("Waiting for response timed out");
            return;
        }
        delay(1);
    }
    
    // Check if the request was successful
    if (tracker != ModbusClient::SUCCESS) {
        Serial.printf("Alarm read failed with status: %s\n", ModbusClient::toString(tracker));
        return;
    }

    // Check if the response has an exception
    if (response.exceptionCode != Modbus::NULL_EXCEPTION) {
        Serial.printf("Modbus exception reading alarms: %s\n", Modbus::toString(response.exceptionCode));
        return;
    }

    // Print all alarm states
    Serial.println("Alarm read complete!");
    for (size_t i = 0; i < response.regCount; i++) {
        Serial.printf("Alarm %d: %s\n", i, response.getCoil(i) ? "ACTIVE" : "inactive");
    }
}

/**
 * Example 3: Synchronous read of setpoints using raw frame
 */
void readSetpoints_Sync() {
    Serial.println("Reading temperature and humidity setpoints...");
    
    // Create frame to read multiple holding registers
    Modbus::Frame request = {
        .type = Modbus::REQUEST,
        .fc = Modbus::READ_HOLDING_REGISTERS,
        .slaveId = THERMOSTAT_SLAVE_ID,
        .regAddress = REG_TEMPERATURE_SETPOINT,
        .regCount = 2,  // Read both temperature and humidity setpoints
        .data = {}
    };
    
    // Send request and wait for response
    // (tracker not provided -> blocks until response received or timeout)
    Modbus::Frame response;
    auto result = client.sendRequest(request, response);
    
    // Check if the request was successful
    if (result != ModbusClient::SUCCESS) {
        Serial.printf("Failed to read setpoints: %s\n", ModbusClient::toString(result));
        return;
    }

    // Check if the response has an exception
    if (response.exceptionCode != Modbus::NULL_EXCEPTION) {
        Serial.printf("Modbus exception reading setpoints: %s\n", Modbus::toString(response.exceptionCode));
        return;
    }

    // Check if the response has the correct number of registers
    if (response.regCount < 2) {
        Serial.println("Invalid response format");
        return;
    }

    // Get the temperature and humidity setpoints from the response
    float tempSetpoint = response.getRegister(0) / 10.0f;
    float humSetpoint = response.getRegister(1) / 10.0f;
    
    // Print the read setpoints
    Serial.printf("Temperature setpoint: %.1f°C\n", tempSetpoint);
    Serial.printf("Humidity setpoint: %.1f%%\n", humSetpoint);
}

/**
 * Example 4: Asynchronous write of setpoints
 */
void writeSetpoints_Async() {
    Serial.println("Writing temperature and humidity setpoints...");
    
    // Create frame to write multiple holding registers
    Modbus::Frame request = {
        .type = Modbus::REQUEST,
        .fc = Modbus::WRITE_MULTIPLE_REGISTERS,
        .slaveId = THERMOSTAT_SLAVE_ID,
        .regAddress = REG_TEMPERATURE_SETPOINT,
        .regCount = 2,  // Write both temperature and humidity setpoints
        .data = Modbus::packRegisters({ 225, 450 })  // temp = 22.5°C, humidity = 45%
    };
    
    // Create frame for response and status tracker
    Modbus::Frame response;
    ModbusClient::Result tracker;
    
    // Send request asynchronously
    // (tracker provided -> returns immediately after transfer started)
    auto result = client.sendRequest(request, response, &tracker);
    
    if (result != ModbusClient::SUCCESS) {
        Serial.printf("Failed to start setpoint write: %s\n", ModbusClient::toString(result));
        return;
    }
    
    Serial.println("Setpoint write request sent. Waiting for completion...");
    
    // Wait for the request to complete
    // (usually this is done in another task/function)
    uint32_t startTime = millis();
    while (tracker == ModbusClient::NODATA) {
        // Timeout after 2 seconds (safety, should be already handled by the ModbusClient)
        if (millis() - startTime > 2000) {
            Serial.println("Waiting for response timed out");
            return;
        }
        delay(1);
    }
    
    // Check if the request was successful
    if (tracker != ModbusClient::SUCCESS) {
        Serial.printf("Setpoint write failed with status: %s\n", ModbusClient::toString(tracker));
        return;
    }

    // Check if the response has an exception
    if (response.exceptionCode != Modbus::NULL_EXCEPTION) {
        Serial.printf("Modbus exception writing setpoints: %s\n", Modbus::toString(response.exceptionCode));
        return;
    }

    // Print the result
    Serial.println("Setpoint write complete! Temperature set to 22.5°C, Humidity set to 45%");
}