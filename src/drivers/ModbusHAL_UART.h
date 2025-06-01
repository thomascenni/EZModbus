/**
 * @file ModbusHAL_UART.h
 * @brief Event-driven HAL wrapper for UART using ESP UART API (header)
 */

#pragma once

#include "core/ModbusCore.h"
#include "utils/ModbusDebug.h"
#include "driver/uart.h" // Contient uart_event_t
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <HardwareSerial.h> // For Arduino compatibility
#endif

namespace ModbusHAL {

class UART {
public:
    // Configuration flags
    static constexpr uint32_t CONFIG_5N1 = UART_DATA_5_BITS | (UART_PARITY_DISABLE << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_5N2 = UART_DATA_5_BITS | (UART_PARITY_DISABLE << 8) | (UART_STOP_BITS_2 << 16);
    static constexpr uint32_t CONFIG_5E1 = UART_DATA_5_BITS | (UART_PARITY_EVEN    << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_5E2 = UART_DATA_5_BITS | (UART_PARITY_EVEN    << 8) | (UART_STOP_BITS_2 << 16);
    static constexpr uint32_t CONFIG_5O1 = UART_DATA_5_BITS | (UART_PARITY_ODD     << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_5O2 = UART_DATA_5_BITS | (UART_PARITY_ODD     << 8) | (UART_STOP_BITS_2 << 16);
    static constexpr uint32_t CONFIG_6N1 = UART_DATA_6_BITS | (UART_PARITY_DISABLE << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_6N2 = UART_DATA_6_BITS | (UART_PARITY_DISABLE << 8) | (UART_STOP_BITS_2 << 16);
    static constexpr uint32_t CONFIG_6E1 = UART_DATA_6_BITS | (UART_PARITY_EVEN    << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_6E2 = UART_DATA_6_BITS | (UART_PARITY_EVEN    << 8) | (UART_STOP_BITS_2 << 16);
    static constexpr uint32_t CONFIG_6O1 = UART_DATA_6_BITS | (UART_PARITY_ODD     << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_6O2 = UART_DATA_6_BITS | (UART_PARITY_ODD     << 8) | (UART_STOP_BITS_2 << 16);
    static constexpr uint32_t CONFIG_7N1 = UART_DATA_7_BITS | (UART_PARITY_DISABLE << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_7N2 = UART_DATA_7_BITS | (UART_PARITY_DISABLE << 8) | (UART_STOP_BITS_2 << 16);
    static constexpr uint32_t CONFIG_7E1 = UART_DATA_7_BITS | (UART_PARITY_EVEN    << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_7E2 = UART_DATA_7_BITS | (UART_PARITY_EVEN    << 8) | (UART_STOP_BITS_2 << 16);
    static constexpr uint32_t CONFIG_7O1 = UART_DATA_7_BITS | (UART_PARITY_ODD     << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_7O2 = UART_DATA_7_BITS | (UART_PARITY_ODD     << 8) | (UART_STOP_BITS_2 << 16);
    static constexpr uint32_t CONFIG_8N1 = UART_DATA_8_BITS | (UART_PARITY_DISABLE << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_8N2 = UART_DATA_8_BITS | (UART_PARITY_DISABLE << 8) | (UART_STOP_BITS_2 << 16);
    static constexpr uint32_t CONFIG_8E1 = UART_DATA_8_BITS | (UART_PARITY_EVEN    << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_8E2 = UART_DATA_8_BITS | (UART_PARITY_EVEN    << 8) | (UART_STOP_BITS_2 << 16);
    static constexpr uint32_t CONFIG_8O1 = UART_DATA_8_BITS | (UART_PARITY_ODD     << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_8O2 = UART_DATA_8_BITS | (UART_PARITY_ODD     << 8) | (UART_STOP_BITS_2 << 16);

    // Internal constants for the UART driver
    static constexpr int MAX_TOUT_THRESH = 102;
    static constexpr int DRIVER_RX_BUFFER_SIZE = 512; 
    static constexpr int DRIVER_TX_BUFFER_SIZE = 256; // Set to 0 for a blocking TX without driver buffer
    static constexpr int DRIVER_EVENT_QUEUE_SIZE = 20; // Will be used internally
    static constexpr int WRITE_TIMEOUT_MS = 1000;
    static constexpr int READ_TIMEOUT_MS = 10; 

    // Vanilla constructor
    UART(uart_port_t uart_num, 
            uint32_t baud_rate, 
            uint32_t config_flags = CONFIG_8N1, 
            int pin_rx = UART_PIN_NO_CHANGE, 
            int pin_tx = UART_PIN_NO_CHANGE, 
            int pin_rts_de = -1); // Accept int, will convert to gpio_num_t internally

    #if defined(ARDUINO_ARCH_ESP32)
    // Arduino constructor
    UART(HardwareSerial& serial_dev,
         uint32_t baud_rate,
         uint32_t arduino_config, // e.g. SERIAL_8N1 from Arduino's HardwareSerial.h
         int pin_rx,
         int pin_tx,
         int pin_rts_de = -1); // Accept int, will convert to gpio_num_t internally
    #endif

    ~UART();

    esp_err_t begin(QueueHandle_t* out_event_queue = nullptr, int intr_alloc_flags = 0);
    void end();

    // Read/write methods
    int read(uint8_t* buf_ptr, size_t max_len_to_read, TickType_t ticks_to_wait = pdMS_TO_TICKS(READ_TIMEOUT_MS));
    size_t write(const uint8_t* buf, size_t size); 
    int available() const;
    esp_err_t flush_input();

    // UART config methods
    uint32_t getBaudrate() const { return _baud_rate; }
    esp_err_t setBaudrate(uint32_t baud_rate); 
    uart_port_t getPort() const { return _uart_num; }
    QueueHandle_t getRegisteredEventQueue() const { return _internal_event_queue_handle; } // Renommé pour clarté

    // Configure UART to support RS485 features
    esp_err_t waitTxComplete(TickType_t timeout_ticks = pdMS_TO_TICKS(WRITE_TIMEOUT_MS)) const;
    esp_err_t setRS485Mode(bool enable);
    esp_err_t getBufferedDataLen(size_t* size) const;
    esp_err_t setTimeoutThreshold(uint8_t timeout_threshold);
    esp_err_t setTimeoutMicroseconds(uint64_t timeout_us);  // Nouvelle méthode
    esp_err_t enablePatternDetection(char pattern_char, uint8_t pattern_length);
    esp_err_t disablePatternDetection();
    esp_err_t flushTxFifo();

private:
    #if defined(ARDUINO_ARCH_ESP32)
    static uint32_t convertArduinoConfig(uint32_t arduino_config); // Convert Arduino config to ESP-IDF config
    #endif

    uart_port_t _uart_num;
    gpio_num_t _pin_rts_de;
    uint32_t _baud_rate;
    uint32_t _config_flags;
    int _pin_tx;
    int _pin_rx;
    QueueHandle_t _internal_event_queue_handle = nullptr;
    bool _is_driver_installed = false;
    uart_config_t _current_hw_config;
};

} // namespace ModbusHAL