/**
 * @file rtu_main.cpp
 * @brief Example using EZModbus for a thermostat application
 * @brief (demonstration of Modbus RTU Client usage)
 */

#include <Arduino.h>
#include "EZModbus.h"

// Aliases for convenience
using UART = ModbusHAL::UART;
using UARTConfig = ModbusHAL::UART::Config;
using ModbusRTU = ModbusInterface::RTU;
using ModbusClient = Modbus::Client;

// Thermostat Modbus configuration
#define THERMOSTAT_SLAVE_ID 1

// Register map for our example thermostat
namespace RegAddr {
    // Coils (read/write)
    constexpr uint16_t REG_TEMP_REGULATION_ENABLE = 100;      // Temperature regulation enable
    
    // Discrete Inputs (read only)
    constexpr uint16_t REG_ALARM_START = 200;                 // 10 discrete inputs for alarms (200-209)
    
    // Input Registers (read only)
    constexpr uint16_t REG_CURRENT_TEMPERATURE = 300;         // Current temperature (°C × 10)
    constexpr uint16_t REG_CURRENT_HUMIDITY = 301;            // Current humidity (% × 10)
    
    // Holding Registers (read/write)
    constexpr uint16_t REG_TEMPERATURE_SETPOINT = 400;        // Temperature setpoint (°C × 10)
    constexpr uint16_t REG_HUMIDITY_SETPOINT = 401;           // Humidity setpoint (% × 10)
}

// UART configuration & instance
UARTConfig uartConfig = {
    .serial = Serial2,
    .baud = 9600,
    .config = SERIAL_8N1,
    .rxPin = 16,
    .txPin = 17,
    .dePin = 5
};
UART uart(uartConfig);

// Modbus RTU interface and client
ModbusRTU interface(uart, Modbus::CLIENT);
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

    // Initialize UART
    uart.begin();
    
    if (client.begin() != ModbusClient::SUCCESS) {
        Serial.println("Failed to initialize Modbus Client");
        while (1) { delay(1000); } // Halt
    }
    
    Serial.println("Modbus client initialized");

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

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// ===================================================================================
// EXAMPLE CASES
// ===================================================================================

/**
 * Example 1: Synchronous read of current temperature and humidity
 */
void readTemperature_Sync() {
    Serial.println("Reading current temperature...");
    
    // Create frame to read temperature (input register)
    Modbus::Frame tempRequest = {
        .type = Modbus::REQUEST,
        .fc = Modbus::READ_INPUT_REGISTERS,
        .slaveId = THERMOSTAT_SLAVE_ID,
        .regAddress = RegAddr::REG_CURRENT_TEMPERATURE,
        .regCount = 1,
        .data = {}
    };
    
    // Send request and wait for response 
    // (tracker not provided -> waits until response received or timeout)
    Modbus::Frame tempResponse;
    auto result = client.sendRequest(tempRequest, tempResponse);

    // Check if the request was successful
    if (result != ModbusClient::SUCCESS) {
        Serial.printf("Failed to start temperature read: %s\n", ModbusClient::toString(result));
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
        .regAddress = RegAddr::REG_ALARM_START,
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
        Serial.print("Failed to start alarm read: ");
        Serial.println(ModbusClient::toString(result));
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
        Serial.printf("Failed to start alarm read: %s\n", ModbusClient::toString(tracker));
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
        .regAddress = RegAddr::REG_TEMPERATURE_SETPOINT,
        .regCount = 2,  // Read both temperature and humidity setpoints
        .data = {}
    };
    
    // Send request and wait for response
    // (tracker not provided -> waits until response received or timeout)
    Modbus::Frame response;
    auto result = client.sendRequest(request, response);

    // Check if the request was successful
    if (result != ModbusClient::SUCCESS) {
        Serial.printf("Failed to start setpoint read: %s\n", ModbusClient::toString(result));
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
 * Example 4: Asynchronous write of setpoints using the callback API
 */
void writeSetpoints_Async() {
    Serial.println("Writing temperature and humidity setpoints (callback mode)...");

    // Two variables that we want to update from the callback
    static uint32_t totalUpdates = 0;
    static uint32_t lastUpdateTime = 0;

    // Build frame to write both setpoints (22.5 °C & 45 % RH)
    Modbus::Frame request = {
        .type       = Modbus::REQUEST,
        .fc         = Modbus::WRITE_MULTIPLE_REGISTERS,
        .slaveId    = THERMOSTAT_SLAVE_ID,
        .regAddress = RegAddr::REG_TEMPERATURE_SETPOINT,
        .regCount   = 2,
        .data       = Modbus::packRegisters({225, 450})
    };

    // Simple context shared with the callback
    struct CbCtx { 
        uint32_t& nb = totalUpdates;
        uint32_t& time = lastUpdateTime;
    } ctx;

    // Static, non-capturing lambda -> decays to a function pointer
    static auto cb = [](ModbusClient::Result res, const Modbus::Frame* resp, void* ctx) {
        auto* c = static_cast<CbCtx*>(ctx);

        if (res == ModbusClient::SUCCESS && resp && resp->exceptionCode == Modbus::NULL_EXCEPTION) {
            Serial.println("Callback: write SUCCESS!");
        } else {
            Serial.printf("Callback: write FAILED (%s)\n", ModbusClient::toString(res));
        }
        
        if (c) {
            c->nb++;
            c->time = millis();
        }
    };

    // Launch request (returns immediately)
    auto result = client.sendRequest(request, cb, &ctx);
    if (result != ModbusClient::SUCCESS) {
        Serial.printf("Failed to queue write request: %s\n", ModbusClient::toString(result));
        return;
    }

    // Fire & forget: no need to wait for the request to complete!
    // The callback will handle everything in the background
}