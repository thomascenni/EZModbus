/**
 * @file rtu_main.cpp
 * @brief Example using EZModbus for a smart greenhouse control system
 * @brief (demonstration of Modbus RTU Server usage with different register types)
 */

#include <Arduino.h>
#include "EZModbus.h"

// Serial port used for Modbus RTU
#define RS485_SERIAL Serial2
#define RS485_BAUD_RATE 9600
#define RS485_CONFIG SERIAL_8N1
#define RS485_RX_PIN 16
#define RS485_TX_PIN 17
#define RS485_DE_PIN 5  // DE/RE pin for RS485 communication

// Server configuration
#define GREENHOUSE_SLAVE_ID 1

// Simulated sensors/actuators pins
const int TEMP_SENSOR_PIN = 36;    // Temperature sensor
const int HUMID_SENSOR_PIN = 39;   // Humidity sensor
const int LIGHT_SENSOR_PIN = 34;   // Light sensor
const int FAN_PIN = 16;            // Ventilation fan
const int PUMP_PIN = 17;           // Water pump
const int LIGHT_PIN = 18;          // Growing lights

// Register map configuration
// Note: All temperatures are in °C × 10, humidity in % × 10
namespace RegAddr {
    // Input registers (read-only, sensor values)
    constexpr uint16_t TEMPERATURE = 100;     // Current temperature
    constexpr uint16_t HUMIDITY = 101;        // Current humidity
    constexpr uint16_t LIGHT_LEVEL = 102;     // Current light level (0-1000)
    constexpr uint16_t WATER_LEVEL = 103;     // Water tank level (0-100%)
    
    // Holding registers (read/write, setpoints and configuration)
    constexpr uint16_t TEMP_SETPOINT = 200;   // Temperature setpoint
    constexpr uint16_t HUMID_SETPOINT = 201;  // Humidity setpoint
    constexpr uint16_t LIGHT_THRESHOLD = 202; // Light threshold for auto-lighting
    
    // Coils (read/write, actuator states and modes)
    constexpr uint16_t FAN_STATE = 300;       // Fan state (ON/OFF)
    constexpr uint16_t PUMP_STATE = 301;      // Pump state (ON/OFF)
    constexpr uint16_t LIGHT_STATE = 302;     // Light state (ON/OFF)
    constexpr uint16_t AUTO_MODE = 303;       // Automatic control mode
    
    // Discrete inputs (read-only, alarms and status)
    constexpr uint16_t TEMP_ALARM = 400;      // Temperature out of range
    constexpr uint16_t HUMID_ALARM = 401;     // Humidity out of range
    constexpr uint16_t WATER_LOW = 402;       // Water level low
}

// Aliases for convenience
using UART = ModbusHAL::UART;
using ModbusRTU = ModbusInterface::RTU;
using ModbusServer = Modbus::Server;
using ModbusRegister = Modbus::Server::Register;

// UART port for Modbus RTU
UART uart(RS485_SERIAL, RS485_BAUD_RATE, RS485_CONFIG, RS485_RX_PIN, RS485_TX_PIN, RS485_DE_PIN);

// Modbus RTU interface and server
ModbusRTU interface(uart, Modbus::SERVER);
ModbusServer server(interface, GREENHOUSE_SLAVE_ID);

// Variables for register storage
struct {
    // Input registers
    volatile uint32_t temperature = 0;
    volatile uint32_t humidity = 0;
    volatile uint32_t lightLevel = 0;
    volatile uint32_t waterLevel = 100;
    
    // Holding registers
    volatile uint32_t tempSetpoint = 250;   // 25.0°C
    volatile uint32_t humidSetpoint = 600;  // 60.0%
    volatile uint32_t lightThreshold = 300; // 30.0%
    
    // Coils
    volatile uint32_t fanState = 0;
    volatile uint32_t pumpState = 0;
    volatile uint32_t lightState = 0;
    volatile uint32_t autoMode = 1;         // Auto mode enabled by default
    
    // Discrete inputs
    volatile uint32_t tempAlarm = 0;
    volatile uint32_t humidAlarm = 0;
    volatile uint32_t waterLowAlarm = 0;

    // Variables pour la gestion des variations max
    volatile uint32_t lastTempSetpoint = 250;
    volatile uint32_t lastHumidSetpoint = 600;
    volatile uint32_t lastSetpointUpdateTime = 0;
} greenhouse;

// Function prototypes
void setupRegisters();
void updateSensors();
void updateActuators();
void checkAlarms();

// Utility function to log register write access
void logRegisterWrite(const ModbusRegister& ctx, uint16_t value, const char* status = nullptr) {
    Serial.print("Write to ");
    Serial.print(ctx.name);
    Serial.print(" (addr: ");
    Serial.print(ctx.address);
    if (status) {
        Serial.print("): ");
        Serial.println(status);
    } else {
        Serial.print("): ");
        Serial.println(value);
    }
}

void setup() {
    // Initialize serial for debug output
    Serial.begin(115200);
    Serial.println("\nEZModbus Greenhouse Server Example");
    
    // Initialize GPIO
    pinMode(FAN_PIN, OUTPUT);
    pinMode(PUMP_PIN, OUTPUT);
    pinMode(LIGHT_PIN, OUTPUT);
    pinMode(TEMP_SENSOR_PIN, INPUT);
    pinMode(HUMID_SENSOR_PIN, INPUT);
    pinMode(LIGHT_SENSOR_PIN, INPUT);

    // Initialize UART driver
    uart.begin();
    
    // Initialize Modbus Server
    if (server.begin() != ModbusServer::SUCCESS) {
        Serial.println("Failed to initialize Modbus Server");
        while (1) { delay(1000); } // Halt
    }
    
    Serial.println("Modbus server initialized");
    
    // Setup Modbus registers
    setupRegisters();
}

void loop() {
    // Update sensors, actuators and alarms
    updateSensors();
    if (greenhouse.autoMode) {
        updateActuators();
    }
    checkAlarms();
    
    // Small delay to prevent too frequent updates
    delay(100);
}

// Register setup with different methods (direct pointer and callbacks)
void setupRegisters() {
    // Pre-reserve register counts for better memory management
    server.setRegisterCount(Modbus::INPUT_REGISTER, 4);    // 4 input registers
    server.setRegisterCount(Modbus::HOLDING_REGISTER, 3);  // 3 holding registers
    server.setRegisterCount(Modbus::COIL, 4);             // 4 coils
    server.setRegisterCount(Modbus::DISCRETE_INPUT, 3);    // 3 discrete inputs
    
    // Input registers - Using direct pointer method
    server.addRegister({
        .type = Modbus::INPUT_REGISTER,
        .address = RegAddr::TEMPERATURE,
        .name = "Temperature",
        .value = &greenhouse.temperature
    });
    
    server.addRegister({
        .type = Modbus::INPUT_REGISTER,
        .address = RegAddr::HUMIDITY,
        .name = "Humidity",
        .value = &greenhouse.humidity
    });
    
    server.addRegister({
        .type = Modbus::INPUT_REGISTER,
        .address = RegAddr::LIGHT_LEVEL,
        .name = "Light Level",
        .value = &greenhouse.lightLevel
    });
    
    server.addRegister({
        .type = Modbus::INPUT_REGISTER,
        .address = RegAddr::WATER_LEVEL,
        .name = "Water Level",
        .value = &greenhouse.waterLevel
    });
    
    // Holding registers - Using callback method for validation
    // & using the register context to log write access
    server.addRegister({
        .type = Modbus::HOLDING_REGISTER,
        .address = RegAddr::TEMP_SETPOINT,
        .name = "Temperature Setpoint",
        .readCb = [](const ModbusRegister& ctx) { 
            return greenhouse.tempSetpoint; 
        },
        .writeCb = [](uint16_t value, const ModbusRegister& ctx) {
            // Validate temperature range (10°C to 35°C)
            if (value < 100 || value > 350) {
                logRegisterWrite(ctx, value, "value out of range");
                return false;
            }

            // Limit the variation to 2°C per minute
            uint32_t currentTime = millis();
            if (currentTime - greenhouse.lastSetpointUpdateTime >= 1000) {  // Check every second
                int16_t maxChange = 20;  // 2.0°C
                int16_t requestedChange = abs(value - (int16_t)greenhouse.lastTempSetpoint);
                
                if (requestedChange > maxChange) {
                    logRegisterWrite(ctx, value, "change too large (max 2°C/min)");
                    return false;
                }
                
                greenhouse.tempSetpoint = value;
                greenhouse.lastTempSetpoint = value;
                greenhouse.lastSetpointUpdateTime = currentTime;
                
                logRegisterWrite(ctx, value);
                return true;
            }
            return false;
        }
    });
    
    server.addRegister({
        .type = Modbus::HOLDING_REGISTER,
        .address = RegAddr::HUMID_SETPOINT,
        .name = "Humidity Setpoint",
        .readCb = [](const ModbusRegister& ctx) { 
            return greenhouse.humidSetpoint; 
        },
        .writeCb = [](uint16_t value, const ModbusRegister& ctx) {
            // Validate humidity range (30% to 90%)
            if (value < 300 || value > 900) {
                logRegisterWrite(ctx, value, "value out of range");
                return false;
            }

            // Limit the variation to 5% per minute
            uint32_t currentTime = millis();
            if (currentTime - greenhouse.lastSetpointUpdateTime >= 1000) {  // Check every second
                int16_t maxChange = 50;  // 5.0%
                int16_t requestedChange = abs(value - (int16_t)greenhouse.lastHumidSetpoint);
                
                if (requestedChange > maxChange) {
                    logRegisterWrite(ctx, value, "change too large (max 5%/min)");
                    return false;
                }
                
                greenhouse.humidSetpoint = value;
                greenhouse.lastHumidSetpoint = value;
                greenhouse.lastSetpointUpdateTime = currentTime;
                
                logRegisterWrite(ctx, value);
                return true;
            }
            return false;
        }
    });
    
    server.addRegister({
        .type = Modbus::HOLDING_REGISTER,
        .address = RegAddr::LIGHT_THRESHOLD,
        .name = "Light Threshold",
        .readCb = [](const ModbusRegister& ctx) { 
            return greenhouse.lightThreshold; 
        },
        .writeCb = [](uint16_t value, const ModbusRegister& ctx) {
            // Validate light threshold (0% to 100%)
            if (value <= 1000) {
                greenhouse.lightThreshold = value;
                return true;
            }
            return false;
        }
    });
    
    // Coils - Mix of direct pointer and callback methods
    server.addRegister({
        .type = Modbus::COIL,
        .address = RegAddr::FAN_STATE,
        .name = "Fan State",
        .value = &greenhouse.fanState
    });
    
    server.addRegister({
        .type = Modbus::COIL,
        .address = RegAddr::PUMP_STATE,
        .name = "Pump State",
        .value = &greenhouse.pumpState
    });
    
    server.addRegister({
        .type = Modbus::COIL,
        .address = RegAddr::LIGHT_STATE,
        .name = "Light State",
        .value = &greenhouse.lightState
    });
    
    // Auto mode with callback for additional actions
    server.addRegister({
        .type = Modbus::COIL,
        .address = RegAddr::AUTO_MODE,
        .name = "Auto Mode",
        .readCb = [](const ModbusRegister& ctx) { 
            return greenhouse.autoMode; 
        },
        .writeCb = [](uint16_t value, const ModbusRegister& ctx) {
            greenhouse.autoMode = value;
            // If auto mode is disabled, keep current actuator states
            // If enabled, force an immediate update
            if (value) {
                updateActuators();
            }
            return true;
        }
    });
    
    // Discrete inputs - Using direct pointer method
    server.addRegister({
        .type = Modbus::DISCRETE_INPUT,
        .address = RegAddr::TEMP_ALARM,
        .name = "Temperature Alarm",
        .value = &greenhouse.tempAlarm
    });
    
    server.addRegister({
        .type = Modbus::DISCRETE_INPUT,
        .address = RegAddr::HUMID_ALARM,
        .name = "Humidity Alarm",
        .value = &greenhouse.humidAlarm
    });
    
    server.addRegister({
        .type = Modbus::DISCRETE_INPUT,
        .address = RegAddr::WATER_LOW,
        .name = "Water Low Alarm",
        .value = &greenhouse.waterLowAlarm
    });
}

// Simulated sensor readings
void updateSensors() {
    // Simulate temperature reading (20°C to 30°C)
    static float temp = 25.0;
    temp += random(-10, 11) / 100.0;
    temp = constrain(temp, 20.0, 30.0);
    greenhouse.temperature = temp * 10;
    
    // Simulate humidity reading (40% to 80%)
    static float humid = 60.0;
    humid += random(-10, 11) / 100.0;
    humid = constrain(humid, 40.0, 80.0);
    greenhouse.humidity = humid * 10;
    
    // Simulate light level (0-1000)
    greenhouse.lightLevel = random(0, 1001);
    
    // Simulate water level depletion
    if (greenhouse.pumpState && greenhouse.waterLevel > 0) {
        greenhouse.waterLevel--;
    }
}

// Update actuators based on sensor values in auto mode
void updateActuators() {
    // Temperature control
    if (greenhouse.temperature > greenhouse.tempSetpoint + 10) { // +1.0°C
        greenhouse.fanState = 1;
    } else if (greenhouse.temperature < greenhouse.tempSetpoint - 10) { // -1.0°C
        greenhouse.fanState = 0;
    }
    
    // Humidity control
    if (greenhouse.humidity < greenhouse.humidSetpoint - 50) { // -5.0%
        greenhouse.pumpState = 1;
    } else if (greenhouse.humidity > greenhouse.humidSetpoint + 50) { // +5.0%
        greenhouse.pumpState = 0;
    }
    
    // Light control
    greenhouse.lightState = (greenhouse.lightLevel < greenhouse.lightThreshold);
    
    // Update physical outputs
    digitalWrite(FAN_PIN, greenhouse.fanState);
    digitalWrite(PUMP_PIN, greenhouse.pumpState);
    digitalWrite(LIGHT_PIN, greenhouse.lightState);
}

// Check and update alarm states
void checkAlarms() {
    // Temperature alarm if more than 2°C from setpoint
    greenhouse.tempAlarm = abs((int)greenhouse.temperature - (int)greenhouse.tempSetpoint) > 20;
    
    // Humidity alarm if more than 10% from setpoint
    greenhouse.humidAlarm = abs((int)greenhouse.humidity - (int)greenhouse.humidSetpoint) > 100;
    
    // Water low alarm at 10%
    greenhouse.waterLowAlarm = (greenhouse.waterLevel <= 10);
}
