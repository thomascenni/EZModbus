/**
 * @file ModbusHAL_TCP.h
 * @brief Hardware Abstraction Layer for TCP sockets using native ESP-IDF
 *
 * Provides a drop-in replacement for Arduino's WiFiClient/WiFiServer
 * for use with EZModbus's ModbusInterface::TCP template class.
 */

#pragma once

#include "core/ModbusCore.h"
#include "utils/ModbusDebug.h"

#include <sys/socket.h>
#include <netdb.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <arpa/inet.h>
#include "esp_netif.h"
#include "esp_log.h"
#include <string>

namespace ModbusHAL {

// Unified TCP client/server interface as per guidelines_hal_tcp.md
class TCP {
public:
    enum CfgMode { UNINIT, SERVER, CLIENT };

    // Technical constraints from guidelines_hal_tcp.md
    static constexpr size_t MAX_ACTIVE_SOCKETS = 4;
    static constexpr size_t MAX_MODBUS_FRAME_SIZE = 260;  // Modbus TCP max frame size (MBAP + PDU)
    static constexpr size_t RX_QUEUE_SIZE = 16; // Number of Modbus frames the RX queue can hold

    // Structure for messages exchanged between HAL and Modbus layer
    struct TCPMsg {
        uint8_t payload[MAX_MODBUS_FRAME_SIZE];
        size_t len;
        int socketNum;  // Socket descriptor: source (RX) or destination (TX)
                        // Using int as socket descriptors are typically integers.
    };

    TCP();
    // Nouveau : configuration directe à la construction
    explicit TCP(uint16_t serverPort);                       // Serveur
    TCP(const char* serverIP, uint16_t port);                // Client
    ~TCP();

    // Disable copy and assign
    TCP(const TCP&) = delete;
    TCP& operator=(const TCP&) = delete;

    // Setup methods
    bool begin(); // utilise les paramètres fournis au ctor
    bool beginServer(uint16_t port, uint32_t ip = INADDR_ANY);
    bool beginClient(const char* serverIP, uint16_t port);
    void stop(); // To close sockets and stop the task

    // Main API
    bool sendMsg(const uint8_t* payload, const size_t len, const int destSocket = -1, int* actualSocket = nullptr);


    // Lecture non bloquante de données sur un socket.
    // 0        : aucune donnée disponible (EAGAIN/EWOULDBLOCK)
    // n (>0)   : n octets copiés dans dst
    // SIZE_MAX : socket fermé ou erreur fatale
    size_t readSocketData(int socketNum, uint8_t* dst, size_t maxLen);

    // Monitoring
    size_t getActiveSocketCount();
    bool isServerRunning() const;
    bool isClientConnected();

    // Get HAL configuration mode (server/client/uninit)
    CfgMode getMode() const { return _cfgMode; }

    // Give access to RX queue for QueueSet integration (read only)
    QueueHandle_t getRxQueueHandle() const { return _rxQueue; }

private:
    // Task for handling socket events and data
    static void tcpTask(void* param);
    void runTcpTask(); // Internal method called by the FreeRTOS task

    // Socket management
    bool setupServerSocket(uint16_t port, uint32_t ip);
    bool setupClientSocket(const char* serverIP, uint16_t port);
    void closeSocket(int sock);

    // Internal state
    TaskHandle_t _tcpTaskHandle;
    QueueHandle_t _rxQueue;
    std::vector<int> _activeSockets; // Stores active client sockets for server, or the server socket for client
    int _listenSocket;              // Server listen socket
    int _clientSocket;              // Client mode connected socket
    
    bool _isServer;                 // True if running in server mode
    volatile bool _isRunning;       // Task control flag
    
    // Mutex for protecting shared resources like _activeSockets
    Mutex _socketsMutex; 

    // Config stockée si constructeur paramétré
    CfgMode _cfgMode = CfgMode::UNINIT;
    std::string _cfgIP;
    uint16_t _cfgPort = 0;
};

} // namespace ModbusHAL
