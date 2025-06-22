// main/main.cpp
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "EZModbus.h"

static const char *TAG_APP = "MODBUS_LOOPBACK_APP";
static const char *TAG_CLIENT_TASK = "CLIENT_TASK";

// Aliases for convenience
using UART = ModbusHAL::UART;
using UARTConfig = ModbusHAL::UART::Config;
using ModbusRTU = ModbusInterface::RTU;
using ModbusClient = Modbus::Client;
using ModbusServer = Modbus::Server;
using ModbusWord = Modbus::Word;

// UART Server configuration
UARTConfig uartServerConfig = {
    .uartNum = UART_NUM_1,
    .baud = 115200,
    .config = UART::CONFIG_8N1,
    .rxPin = GPIO_NUM_44,
    .txPin = GPIO_NUM_7,
    .dePin = GPIO_NUM_NC
};

// UART Client configuration
UARTConfig uartClientConfig = {
    .uartNum = UART_NUM_2,
    .baud = 115200,
    .config = UART::CONFIG_8N1,
    .rxPin = GPIO_NUM_6,
    .txPin = GPIO_NUM_43,
    .dePin = GPIO_NUM_NC
};

// Common Modbus RTU cfg
#define SERVER_SLAVE_ID         1
#define TARGET_REGISTER         100
#define NUM_WORDS               1 // Number of registers to read/write
#define CLIENT_POLL_INTERVAL_MS 2000


// MODBUS INSTANCES

// Server
UART uartServer(uartServerConfig);
ModbusRTU rtuServer(uartServer, Modbus::SERVER);
Modbus::StaticWordStore<NUM_WORDS> store;
ModbusServer modbusServer(rtuServer, store, SERVER_SLAVE_ID);

// Client
UART uartClient(uartClientConfig);
ModbusRTU rtuClient(uartClient, Modbus::CLIENT);
ModbusClient modbusClient(rtuClient);

// Server variable
volatile uint16_t counter = 1000;

void clientTask(void *pvParameters) {
    ESP_LOGI(TAG_CLIENT_TASK, "Client Modbus task started.");
    uint16_t valueToWrite = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CLIENT_POLL_INTERVAL_MS));

        if (!modbusClient.isReady()) {
            ESP_LOGW(TAG_CLIENT_TASK, "Modbus client not ready.");
            continue;
        }

        Modbus::Frame readRequest;
        readRequest.type = Modbus::REQUEST;
        readRequest.fc = Modbus::READ_HOLDING_REGISTERS;
        readRequest.slaveId = SERVER_SLAVE_ID;
        readRequest.regAddress = TARGET_REGISTER;
        readRequest.regCount = 1;

        Modbus::Frame readResponse;
        ESP_LOGI(TAG_CLIENT_TASK, "Sending READ request for register %u...", TARGET_REGISTER);
        ModbusClient::Result readResult = modbusClient.sendRequest(readRequest, readResponse);

        if (readResult != ModbusClient::SUCCESS) {
            ESP_LOGE(TAG_CLIENT_TASK, "Error on sendRequest (READ): %s", ModbusClient::toString(readResult));
            continue;
        }

        if (readResponse.exceptionCode != Modbus::NULL_EXCEPTION) {
            ESP_LOGE(TAG_CLIENT_TASK, "Modbus Exception on READ: %s (0x%02X)",
                     Modbus::toString(readResponse.exceptionCode), readResponse.exceptionCode);
            continue;
        }

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

        if (writeResult != ModbusClient::SUCCESS) {
            ESP_LOGE(TAG_CLIENT_TASK, "Error on sendRequest (WRITE): %s", ModbusClient::toString(writeResult));
            continue;
        }

        if (writeResponse.exceptionCode != Modbus::NULL_EXCEPTION) {
            ESP_LOGE(TAG_CLIENT_TASK, "Modbus Exception on WRITE: %s (0x%02X)",
                     Modbus::toString(writeResponse.exceptionCode), writeResponse.exceptionCode);
            continue;
        }

        ESP_LOGI(TAG_CLIENT_TASK, "WRITE response: Success (Echo Addr: %u, Val: %u)",
                 writeResponse.regAddress, writeResponse.getRegister(0));
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
    ESP_LOGI(TAG_APP, "Server UART HAL (UART) initialized.");

    esp_err_t uartClientInitResult = uartClient.begin();
    if (uartClientInitResult != ESP_OK) {
        ESP_LOGE(TAG_APP, "Error initializing Client UART HAL: %s", esp_err_to_name(uartClientInitResult));
        return;
    }
    ESP_LOGI(TAG_APP, "Client UART HAL (UART) initialized.");

    ModbusWord regDesc = {
        .type = Modbus::HOLDING_REGISTER,
        .startAddr = TARGET_REGISTER,
        .nbRegs = 1,
        .value = &counter
    };
    ModbusServer::Result addWordResult = modbusServer.addWord(regDesc);
    if (addWordResult != ModbusServer::SUCCESS) {
        ESP_LOGE(TAG_APP, "Error adding word to server: %s", ModbusServer::toString(addWordResult));
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