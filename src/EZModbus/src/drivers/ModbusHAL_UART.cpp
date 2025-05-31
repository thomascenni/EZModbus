#include "ModbusHAL_UART.h"

#if defined(ARDUINO_ARCH_ESP32)
    #include <Arduino.h> // Provides HardwareSerial and SERIAL_ defines
#endif

namespace ModbusHAL {

static const char* HAL_TAG = "ModbusHAL_UART";

// Helper pour décoder les flags de config
static void decode_config_flags(uint32_t flags, uart_word_length_t& data_bits, uart_parity_t& parity, uart_stop_bits_t& stop_bits) {
    data_bits = (uart_word_length_t)(flags & 0xFF);
    parity = (uart_parity_t)((flags >> 8) & 0xFF);
    stop_bits = (uart_stop_bits_t)((flags >> 16) & 0xFF);
}

// ---------- EzmUart Implementation ----------
UART::UART(uart_port_t uart_num, 
                 uint32_t baud_rate, 
                 uint32_t config_flags, 
                 int pin_rx, 
                 int pin_tx, 
                 int pin_rts_de)
    : _uart_num(uart_num), 
      _pin_rts_de((pin_rts_de == -1) ? GPIO_NUM_NC : static_cast<gpio_num_t>(pin_rts_de)), 
      _baud_rate(baud_rate),
      _config_flags(config_flags),
      _pin_tx(pin_tx),
      _pin_rx(pin_rx),
      _is_driver_installed(false) {
        // Initialisation de _current_hw_config avec les paramètres fournis
        decode_config_flags(_config_flags, _current_hw_config.data_bits, _current_hw_config.parity, _current_hw_config.stop_bits);
        _current_hw_config.baud_rate = _baud_rate;
        _current_hw_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
            _current_hw_config.source_clk = UART_SCLK_DEFAULT;
        #else
            _current_hw_config.source_clk = UART_SCLK_APB;
        #endif
        Modbus::Debug::LOG_MSG(std::string("Constructor for port ") + std::to_string(_uart_num));
}

#if defined(ARDUINO_ARCH_ESP32)

uint32_t UART::convertArduinoConfig(uint32_t arduino_config) {
    switch (arduino_config) {
        // 8-bit data
        case SERIAL_8N1: return CONFIG_8N1;
        case SERIAL_8N2: return CONFIG_8N2;
        case SERIAL_8E1: return CONFIG_8E1;
        case SERIAL_8E2: return CONFIG_8E2;
        case SERIAL_8O1: return CONFIG_8O1;
        case SERIAL_8O2: return CONFIG_8O2;
        // 7-bit data
        case SERIAL_7N1: return CONFIG_7N1;
        case SERIAL_7N2: return CONFIG_7N2;
        case SERIAL_7E1: return CONFIG_7E1;
        case SERIAL_7E2: return CONFIG_7E2;
        case SERIAL_7O1: return CONFIG_7O1;
        case SERIAL_7O2: return CONFIG_7O2;
        // 6-bit data
        case SERIAL_6N1: return CONFIG_6N1;
        case SERIAL_6N2: return CONFIG_6N2;
        case SERIAL_6E1: return CONFIG_6E1;
        case SERIAL_6E2: return CONFIG_6E2;
        case SERIAL_6O1: return CONFIG_6O1;
        case SERIAL_6O2: return CONFIG_6O2;
        // 5-bit data
        case SERIAL_5N1: return CONFIG_5N1;
        case SERIAL_5N2: return CONFIG_5N2;
        case SERIAL_5E1: return CONFIG_5E1;
        case SERIAL_5E2: return CONFIG_5E2;
        case SERIAL_5O1: return CONFIG_5O1;
        case SERIAL_5O2: return CONFIG_5O2;
        default:
            ESP_LOGW(HAL_TAG, "Unknown Arduino UART config: 0x%X. Defaulting to 8N1.", arduino_config);
            return CONFIG_8N1;
    }
}

UART::UART(HardwareSerial& serial_dev,
             uint32_t baud_rate,
             uint32_t arduino_config,
             int pin_rx,
             int pin_tx,
             int pin_rts_de)
    // Determine uart_port_t by comparing with global Serial, Serial1, Serial2 objects
    // then delegate to the existing constructor.
    : UART(
          // Determine uart_port_t
          ([&]() -> uart_port_t { // Added explicit return type for clarity
              // Ensure Serial, Serial1, Serial2 are declared (they are in HardwareSerial.h)
              // We need to include Arduino.h or HardwareSerial.h to get their declarations.
              // It's already included at the top of this .cpp file inside #if defined(ARDUINO_ARCH_ESP32)
              
              // Serial0 always exists and is a HardwareSerial instance.
              if (&serial_dev == &Serial0) {
                  return UART_NUM_0;
              }

#if SOC_UART_NUM > 1 // Check if Serial1 is defined (ESP32 has at least 2 UARTs)
              // Serial1 is declared in HardwareSerial.h only if SOC_UART_NUM > 1.
              if (&serial_dev == &Serial1) {
                  return UART_NUM_1;
              }
#endif

#if SOC_UART_NUM > 2 // Check if Serial2 is defined (ESP32 has 3 UARTs)
              // Serial2 is declared in HardwareSerial.h only if SOC_UART_NUM > 2.
              if (&serial_dev == &Serial2) {
                  return UART_NUM_2;
              }
#endif
              // Fallback or error if it's not one of the global objects.
              // This situation should ideally not happen if users pass Serial0, Serial1, or Serial2.
              // If they create a HardwareSerial instance manually, e.g. HardwareSerial mySerial(X);
              // then they should use the other constructor of ModbusHAL::UART directly with the port number.
              ESP_LOGE(HAL_TAG, "Could not identify UART port from the provided HardwareSerial object. "
                                "It was not identified as Serial0, Serial1, or Serial2. "
                                "Defaulting to UART_NUM_0. Consider using the constructor that accepts a uart_port_t.");
              return UART_NUM_0; // Fallback to UART_NUM_0 with a warning. Consider if another action is better.
          })(),
          baud_rate,
          convertArduinoConfig(arduino_config),
          pin_tx,
          pin_rx,
          pin_rts_de) // Pass int directly, conversion handled by the delegated constructor
{
    // The actual initialization is done by the delegated constructor.
    Modbus::Debug::LOG_MSG(std::string("Arduino-compatible constructor for UART port ") + std::to_string(getPort()) /* Use getPort() now as _uart_num is set by delegated ctor*/ );
}
#endif // ARDUINO_ARCH_ESP32

UART::~UART() {
    if (_is_driver_installed) {
        end();
    }
    Modbus::Debug::LOG_MSG(std::string("Destructor for port ") + std::to_string(_uart_num));
}

esp_err_t UART::begin(QueueHandle_t* out_event_queue, int intr_alloc_flags) {
    if (_is_driver_installed) {
        Modbus::Debug::LOG_MSG(std::string("Warning: Port ") + std::to_string(_uart_num) + " already initialized. Call end() first.");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = uart_param_config(_uart_num, &_current_hw_config);
    if (err != ESP_OK) {
        Modbus::Debug::LOG_MSG(std::string("Error: uart_param_config failed for port ") + std::to_string(_uart_num) + ": " + esp_err_to_name(err));
        return err;
    }

    int rts_pin_to_set = (_pin_rts_de == GPIO_NUM_NC) ? UART_PIN_NO_CHANGE : _pin_rts_de;
    err = uart_set_pin(_uart_num, _pin_tx, _pin_rx, rts_pin_to_set, UART_PIN_NO_CHANGE /*CTS*/);
    if (err != ESP_OK) {
        Modbus::Debug::LOG_MSG(std::string("Error: uart_set_pin FAILED for port ") + std::to_string(_uart_num) + " with TX:" + std::to_string(_pin_tx) + " RX:" + std::to_string(_pin_rx) + " RTS:" + std::to_string(rts_pin_to_set) + " Error: " + esp_err_to_name(err));
        return err;
    }

    err = uart_driver_install(_uart_num, DRIVER_RX_BUFFER_SIZE, DRIVER_TX_BUFFER_SIZE, 
                              DRIVER_EVENT_QUEUE_SIZE, 
                              &_internal_event_queue_handle, 
                              intr_alloc_flags);
    if (err != ESP_OK) {
        Modbus::Debug::LOG_MSG(std::string("Error: uart_driver_install failed for port ") + std::to_string(_uart_num) + ": " + esp_err_to_name(err));
        _internal_event_queue_handle = nullptr; 
        return err;
    }
    
    if (!_internal_event_queue_handle && DRIVER_EVENT_QUEUE_SIZE > 0) { 
         Modbus::Debug::LOG_MSG(std::string("Error: uart_driver_install succeeded but queue handle is null for port ") + std::to_string(_uart_num));
         return ESP_FAIL; 
    }

    // Configure RS485 mode if RTS/DE pin is specified
    if (_pin_rts_de != GPIO_NUM_NC) {
        err = setRS485Mode(true);
        if (err != ESP_OK) {
            Modbus::Debug::LOG_MSG(std::string("Error: Failed to set RS485 mode for port ") + std::to_string(_uart_num) + ": " + esp_err_to_name(err));
            uart_driver_delete(_uart_num);
            _internal_event_queue_handle = nullptr;
            _is_driver_installed = false;
            return err;
        }
        Modbus::Debug::LOG_MSG(std::string("Port ") + std::to_string(_uart_num) + " configured for RS485 Half-Duplex with DE on pin " + std::to_string((int)_pin_rts_de));
    }
    
    _is_driver_installed = true;
    Modbus::Debug::LOG_MSG(std::string("Port ") + std::to_string(_uart_num) + " initialized. Baud: " + std::to_string((int)_baud_rate) + 
                         ", Config: 0x" + std::to_string((unsigned int)_config_flags) + ", TX:" + std::to_string(_pin_tx) + 
                         ", RX:" + std::to_string(_pin_rx) + ", DE:" + std::to_string((int)_pin_rts_de));
    return ESP_OK;
}

void UART::end() {
    if (_is_driver_installed) {
        esp_err_t err = uart_driver_delete(_uart_num);
        if (err != ESP_OK) {
            Modbus::Debug::LOG_MSG(std::string("Error: uart_driver_delete failed for port ") + std::to_string(_uart_num) + ": " + esp_err_to_name(err));
        }
        _is_driver_installed = false;
        _internal_event_queue_handle = nullptr; 
        Modbus::Debug::LOG_MSG(std::string("Port ") + std::to_string(_uart_num) + " de-initialized.");
    }
}

// Nouvelle implémentation générique
int UART::read(uint8_t* buf_ptr, size_t max_len_to_read, TickType_t ticks_to_wait) {
    if (!_is_driver_installed || !buf_ptr || max_len_to_read == 0) {
        // On pourrait logguer une erreur ici si désiré, ex: ESP_LOGE(HAL_TAG, "EzmUart::read invalid params or driver not installed");
        return -1; // Convention d'erreur: -1 pour paramètres invalides ou driver non prêt
    }
    
    // uart_read_bytes retourne le nombre d'octets lus, ou -1 en cas d'erreur ESP spécifique.
    // Elle peut aussi retourner 0 si le timeout expire avant que des données soient lues.
    int bytes_read = uart_read_bytes(_uart_num, buf_ptr, max_len_to_read, ticks_to_wait);
    
    // if (bytes_read < 0) {
    //     // Logguer l'erreur spécifique de uart_read_bytes si besoin
    //     // ESP_LOGE(HAL_TAG, "uart_read_bytes error: %d on port %d", bytes_read, _uart_num);
    // }
    return bytes_read;
}

size_t UART::write(const uint8_t* buf, size_t size) {
    if (!_is_driver_installed || size == 0) return 0;
    
    int sent_by_driver = uart_write_bytes(_uart_num, (const char*)buf, size);
    
    if (sent_by_driver < 0) { 
        Modbus::Debug::LOG_MSG(std::string("Error: uart_write_bytes error on port ") + std::to_string(_uart_num) + ": " + std::to_string(sent_by_driver));
        return 0; 
    }
    if ((size_t)sent_by_driver < size) {
        Modbus::Debug::LOG_MSG(std::string("Warning: uart_write_bytes partial write on port ") + std::to_string(_uart_num) + ". Requested " + std::to_string(size) + ", sent to buffer " + std::to_string(sent_by_driver));
        // Ne pas continuer si tout n'a pas été accepté par le buffer/FIFO TX
        return (size_t)sent_by_driver; // Ou retourner 0 pour indiquer un échec partiel ? Pour l'instant, on retourne ce qui a été bufferisé.
    }

    // Attendre que la transmission physique soit terminée
    esp_err_t tx_done_err = uart_wait_tx_done(_uart_num, pdMS_TO_TICKS(WRITE_TIMEOUT_MS)); 
    if (tx_done_err == ESP_ERR_TIMEOUT) {
        Modbus::Debug::LOG_MSG(std::string("Warning: uart_wait_tx_done timed out on port ") + std::to_string(_uart_num) + " after " + std::to_string(WRITE_TIMEOUT_MS) + " ms for " + std::to_string(sent_by_driver) + " bytes");
        return 0; // Échec de confirmation de la transmission physique complète
    } else if (tx_done_err != ESP_OK) {
        Modbus::Debug::LOG_MSG(std::string("Error: uart_wait_tx_done failed on port ") + std::to_string(_uart_num) + ": " + esp_err_to_name(tx_done_err));
        return 0; // Échec de confirmation
    }
    
    return (size_t)sent_by_driver; // Succès, tous les octets demandés ont été acceptés et transmis physiquement
}

int UART::available() const {
    if (!_is_driver_installed) return 0;
    size_t len = 0;
    uart_get_buffered_data_len(_uart_num, &len);
    return (int)len;
}

esp_err_t UART::flush_input(){
    if (!_is_driver_installed) return ESP_ERR_INVALID_STATE;
    return uart_flush_input(_uart_num);
}

esp_err_t UART::setBaudrate(uint32_t baud_rate) {
    _baud_rate = baud_rate; 
    _current_hw_config.baud_rate = baud_rate; 

    if (!_is_driver_installed) {
        Modbus::Debug::LOG_MSG(std::string("Info: Port ") + std::to_string(_uart_num) + " baudrate set to " + std::to_string((int)_baud_rate) + " (will be applied at begin())");
        return ESP_OK;
    }
    esp_err_t err = uart_param_config(_uart_num, &_current_hw_config);
    if (err == ESP_OK) {
        Modbus::Debug::LOG_MSG(std::string("Info: Port ") + std::to_string(_uart_num) + " baudrate reconfigured to " + std::to_string((int)_baud_rate));
    } else {
        Modbus::Debug::LOG_MSG(std::string("Error: Port ") + std::to_string(_uart_num) + " failed to reconfigure baudrate to " + std::to_string((int)baud_rate) + ": " + esp_err_to_name(err));
    }
    return err;
}

// Nouvelles implémentations des méthodes RS485

esp_err_t UART::waitTxComplete(TickType_t timeout_ticks) const {
    if (!_is_driver_installed) return ESP_ERR_INVALID_STATE;
    return uart_wait_tx_done(_uart_num, timeout_ticks);
}

esp_err_t UART::setRS485Mode(bool enable) {
    if (!_is_driver_installed) return ESP_ERR_INVALID_STATE;
    if (_pin_rts_de == GPIO_NUM_NC && enable) {
        return ESP_ERR_INVALID_ARG; // Impossible d'activer le mode RS485 sans pin DE/RTS
    }
    uart_mode_t mode = enable ? UART_MODE_RS485_HALF_DUPLEX : UART_MODE_UART;
    return uart_set_mode(_uart_num, mode);
}

esp_err_t UART::getBufferedDataLen(size_t* size) const {
    if (!_is_driver_installed || !size) return ESP_ERR_INVALID_STATE;
    return uart_get_buffered_data_len(_uart_num, size);
}

esp_err_t UART::setTimeoutThreshold(uint8_t timeout_threshold) {
    if (!_is_driver_installed) return ESP_ERR_INVALID_STATE;
    if (timeout_threshold >= MAX_TOUT_THRESH) {
        timeout_threshold = MAX_TOUT_THRESH - 1;
    }
    return uart_set_rx_timeout(_uart_num, timeout_threshold);
}

esp_err_t UART::enablePatternDetection(char pattern_char, uint8_t pattern_length) {
    if (!_is_driver_installed) return ESP_ERR_INVALID_STATE;
    esp_err_t err = uart_enable_pattern_det_baud_intr(_uart_num, pattern_char, pattern_length, 1, 0, 0);
    if (err != ESP_OK) return err;
    return uart_pattern_queue_reset(_uart_num, DRIVER_EVENT_QUEUE_SIZE);
}

esp_err_t UART::disablePatternDetection() {
    if (!_is_driver_installed) return ESP_ERR_INVALID_STATE;
    return uart_disable_pattern_det_intr(_uart_num);
}

esp_err_t UART::flushTxFifo() {
    if (!_is_driver_installed) return ESP_ERR_INVALID_STATE;
    return uart_flush(_uart_num);
}

esp_err_t UART::setTimeoutMicroseconds(uint64_t timeout_us) {
    if (!_is_driver_installed || _baud_rate == 0) return ESP_ERR_INVALID_STATE;

    // Calculer le nombre de symboles UART pour le timeout
    constexpr uint32_t bitsPerUartSymbol = 11;  // 1 start + 8 data + 1 parity + 1 stop
    uint64_t usPerUartSymbol = (static_cast<uint64_t>(bitsPerUartSymbol) * 1000000ULL) / _baud_rate;
    if (usPerUartSymbol == 0) return ESP_ERR_INVALID_ARG;

    // Calculer le threshold en nombre de symboles
    uint8_t threshold = static_cast<uint8_t>((timeout_us + usPerUartSymbol - 1) / usPerUartSymbol);
    
    // Appliquer les limites
    if (threshold == 0) threshold = 1;
    if (threshold >= MAX_TOUT_THRESH) threshold = MAX_TOUT_THRESH - 1;

    // Configurer le timeout
    esp_err_t err = setTimeoutThreshold(threshold);
    if (err == ESP_OK) {
        Modbus::Debug::LOG_MSG(std::string("UART timeout set to: ") + std::to_string(timeout_us) +
                             " us -> " + std::to_string(threshold) + " UART symbols threshold");
    }
    return err;
}

} // namespace ModbusHAL