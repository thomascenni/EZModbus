// main/main.cpp
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "EZModbus.h"

static const char *TAG_APP = "MODBUS_LOOPBACK_APP";
static const char *TAG_SERVER_TASK = "SERVER_TASK";
static const char *TAG_CLIENT_TASK = "CLIENT_TASK";

// UART1 configuration (server)
#define UART_SERVER_PORT    UART_NUM_1
#define UART_SERVER_TX_PIN  GPIO_NUM_7
#define UART_SERVER_RX_PIN  GPIO_NUM_44
#define UART_SERVER_DE_PIN  GPIO_NUM_NC

// UART2 configuration (client)
#define UART_CLIENT_PORT    UART_NUM_2
#define UART_CLIENT_TX_PIN  GPIO_NUM_43
#define UART_CLIENT_RX_PIN  GPIO_NUM_6
#define UART_CLIENT_DE_PIN  GPIO_NUM_NC

// Common Modbus RTU cfg
#define MODBUS_BAUD_RATE    115200
#define MODBUS_CONFIG       ModbusHAL::UART::CONFIG_8N1
#define SERVER_SLAVE_ID     1
#define TARGET_REGISTER     100
#define CLIENT_POLL_INTERVAL_MS 2000

// Aliases for convenience
using UART = ModbusHAL::UART;
using ModbusRTU = ModbusInterface::RTU;
using ModbusClient = Modbus::Client;
using ModbusServer = Modbus::Server;
using ModbusRegister = Modbus::Server::Register;

// MODBUS INSTANCES

// Server
UART uartServer(UART_SERVER_PORT, MODBUS_BAUD_RATE, MODBUS_CONFIG,
                     UART_SERVER_RX_PIN, UART_SERVER_TX_PIN, UART_SERVER_DE_PIN);
ModbusRTU rtuServer(uartServer, Modbus::SERVER);
ModbusServer modbusServer(rtuServer, SERVER_SLAVE_ID);

// Client
UART uartClient(UART_CLIENT_PORT, MODBUS_BAUD_RATE, MODBUS_CONFIG,
                     UART_CLIENT_RX_PIN, UART_CLIENT_TX_PIN, UART_CLIENT_DE_PIN);
ModbusRTU rtuClient(uartClient, Modbus::CLIENT);
ModbusClient modbusClient(rtuClient);

// Server variable
volatile uint32_t counter = 1000;

void clientTask(void *pvParameters) {
    ESP_LOGI(TAG_CLIENT_TASK, "Client Modbus task started.");
    uint16_t valueToWrite = 0;

    while (1) {
        if (modbusClient.isReady()) {
            Modbus::Frame readRequest;
            readRequest.type = Modbus::REQUEST;
            readRequest.fc = Modbus::READ_HOLDING_REGISTERS;
            readRequest.slaveId = SERVER_SLAVE_ID;
            readRequest.regAddress = TARGET_REGISTER;
            readRequest.regCount = 1;

            Modbus::Frame readResponse;
            ESP_LOGI(TAG_CLIENT_TASK, "Sending READ request for register %u...", TARGET_REGISTER);
            ModbusClient::Result readResult = modbusClient.sendRequest(readRequest, readResponse);

            if (readResult == ModbusClient::SUCCESS) {
                if (readResponse.exceptionCode == Modbus::NULL_EXCEPTION) {
                    uint16_t receivedValue = readResponse.getRegister(0);
                    ESP_LOGI(TAG_CLIENT_TASK, "READ response: Register %u = %u", TARGET_REGISTER, receivedValue);

                    valueToWrite = receivedValue + 1;
                    Modbus::Frame writeRequest;
                    writeRequest.type = Modbus::REQUEST;
                    writeRequest.fc = Modbus::WRITE_REGISTER;
                    writeRequest.slaveId = SERVER_SLAVE_ID;
                    writeRequest.regAddress = TARGET_REGISTER;
                    writeRequest.regCount = 1;
                    writeRequest.setRegisters({valueToWrite});

                    Modbus::Frame writeResponse;
                    ESP_LOGI(TAG_CLIENT_TASK, "Sending WRITE request for register %u with value %u...", TARGET_REGISTER, valueToWrite);
                    ModbusClient::Result writeResult = modbusClient.sendRequest(writeRequest, writeResponse);

                    if (writeResult == ModbusClient::SUCCESS) {
                        if (writeResponse.exceptionCode == Modbus::NULL_EXCEPTION) {
                            ESP_LOGI(TAG_CLIENT_TASK, "WRITE response: Success (Echo Addr: %u, Val: %u)",
                                     writeResponse.regAddress, writeResponse.getRegister(0));
                        } else {
                            ESP_LOGE(TAG_CLIENT_TASK, "Modbus Exception on WRITE: %s (0x%02X)",
                                     Modbus::toString(writeResponse.exceptionCode), writeResponse.exceptionCode);
                        }
                    } else {
                        ESP_LOGE(TAG_CLIENT_TASK, "Error on sendRequest (WRITE): %s", ModbusClient::toString(writeResult));
                    }
                } else {
                    ESP_LOGE(TAG_CLIENT_TASK, "Modbus Exception on READ: %s (0x%02X)",
                             Modbus::toString(readResponse.exceptionCode), readResponse.exceptionCode);
                }
            } else {
                ESP_LOGE(TAG_CLIENT_TASK, "Error on sendRequest (READ): %s", ModbusClient::toString(readResult));
            }
        } else {
            ESP_LOGW(TAG_CLIENT_TASK, "Modbus client not ready.");
        }
        vTaskDelay(pdMS_TO_TICKS(CLIENT_POLL_INTERVAL_MS));
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG_APP, "Starting Modbus RTU Client <-> Server Loopback Test (Strict Aliases)");

    esp_err_t uartServerInitResult = uartServer.begin();
    if (uartServerInitResult != ESP_OK) {
        ESP_LOGE(TAG_APP, "Error initializing Server UART HAL: %s", esp_err_to_name(uartServerInitResult));
        return;
    }
    ESP_LOGI(TAG_APP, "Server UART HAL (UART%d) initialized.", UART_SERVER_PORT);

    esp_err_t uartClientInitResult = uartClient.begin();
    if (uartClientInitResult != ESP_OK) {
        ESP_LOGE(TAG_APP, "Error initializing Client UART HAL: %s", esp_err_to_name(uartClientInitResult));
        return;
    }
    ESP_LOGI(TAG_APP, "Client UART HAL (UART%d) initialized.", UART_CLIENT_PORT);

    ModbusRegister regDesc = {
        .type = Modbus::HOLDING_REGISTER,
        .address = TARGET_REGISTER,
        .name = "LoopbackTestRegister",
        .value = &counter
    };
    ModbusServer::Result addRegResult = modbusServer.addRegister(regDesc);
    if (addRegResult != ModbusServer::SUCCESS) {
        ESP_LOGE(TAG_APP, "Error adding register to server: %s", ModbusServer::toString(addRegResult));
        return;
    }
    ESP_LOGI(TAG_APP, "Register %u added to server with initial value %lu.", TARGET_REGISTER, (unsigned long)counter);

    ModbusServer::Result serverInitResult = modbusServer.begin();
    if (serverInitResult != ModbusServer::SUCCESS) {
        ESP_LOGE(TAG_APP, "Error initializing Modbus Server: %s", ModbusServer::toString(serverInitResult));
        return;
    }
    ESP_LOGI(TAG_APP, "Modbus Server initialized (Slave ID: %d).", SERVER_SLAVE_ID);

    ModbusClient::Result clientInitResult = modbusClient.begin();
    if (clientInitResult != ModbusClient::SUCCESS) {
        ESP_LOGE(TAG_APP, "Error initializing Modbus Client: %s", ModbusClient::toString(clientInitResult));
        return;
    }
    ESP_LOGI(TAG_APP, "Modbus Client initialized.");

    xTaskCreate(clientTask, "modbusClientTask", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG_APP, "Setup complete. Client will send periodic requests.");
    ESP_LOGI(TAG_APP, "Initial server register value (app_main): %lu", (unsigned long)counter);

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG_APP, "Current server register value (app_main loop): %lu", (unsigned long)counter);
    }
}