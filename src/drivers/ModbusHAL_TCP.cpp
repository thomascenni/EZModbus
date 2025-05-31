/**
 * @file ModbusHAL_TCP.cpp
 * @brief Hardware Abstraction Layer for TCP sockets using native ESP-IDF
 *
 * Provides a drop-in replacement for Arduino's WiFiClient/WiFiServer
 * for use with EZModbus's ModbusInterface::TCP template class.
 */

#include "drivers/ModbusHAL_TCP.h"
#include "core/ModbusTypes.h" // For Mutex, Lock, TIME_MS, WAIT_MS

// System / ESP-IDF includes
#include <sys/socket.h>
#include <netdb.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h> // For memset, memcpy
#include <arpa/inet.h> // For htons, inet_pton
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <vector>
#include <algorithm> // For std::max, std::remove, std::remove_if

// Define a TAG for logging
static const char* HAL_TCP_TAG = "ModbusHAL_TCP";

namespace ModbusHAL {

TCP::TCP()
    : _tcpTaskHandle(nullptr),
      _rxQueue(nullptr),
      _listenSocket(-1),
      _clientSocket(-1),
      _isServer(false),
      _isRunning(false),
      _cfgMode(CfgMode::UNINIT),
      _cfgPort(0) {
    ESP_LOGD(HAL_TCP_TAG, "TCP HAL instance created.");
}

TCP::~TCP() {
    ESP_LOGD(HAL_TCP_TAG, "TCP HAL instance being destroyed.");
    stop();
}

void TCP::stop() {
    ESP_LOGI(HAL_TCP_TAG, "Stopping TCP HAL...");
    _isRunning = false;

    if (_tcpTaskHandle) {
        ESP_LOGD(HAL_TCP_TAG, "Waiting for TCP task to terminate...");
        // The task should check _isRunning and delete itself.
        // We give it some time to do so. A more robust way would be to signal it.
        vTaskDelay(pdMS_TO_TICKS(200)); // Wait a bit for the task to self-terminate
        // If task still exists after delay, delete it.
        // However, it's better if task cleans up and deletes itself.
        // For now, assume task checks _isRunning flag.
        _tcpTaskHandle = nullptr; // Task should have deleted itself
    }

    {
        Lock guard(_socketsMutex);
        if (_listenSocket != -1) {
            ESP_LOGD(HAL_TCP_TAG, "Closing listen socket %d", _listenSocket);
            ::close(_listenSocket);
            _listenSocket = -1;
        }
        if (_clientSocket != -1) {
            ESP_LOGD(HAL_TCP_TAG, "Closing client socket %d", _clientSocket);
            ::close(_clientSocket);
            _clientSocket = -1;
        }
        for (int sock : _activeSockets) {
            ESP_LOGD(HAL_TCP_TAG, "Closing active client socket %d", sock);
            ::close(sock);
        }
        _activeSockets.clear();
    }


    if (_rxQueue) {
        ESP_LOGD(HAL_TCP_TAG, "Deleting RX queue.");
        vQueueDelete(_rxQueue);
        _rxQueue = nullptr;
    }
    ESP_LOGI(HAL_TCP_TAG, "TCP HAL stopped.");
}

bool TCP::setupServerSocket(uint16_t port, uint32_t ip) {
    _listenSocket = ::socket(AF_INET, SOCK_STREAM, 0);
    if (_listenSocket < 0) {
        ESP_LOGE(HAL_TCP_TAG, "Failed to create listen socket, errno: %d", errno);
        return false;
    }
    ESP_LOGD(HAL_TCP_TAG, "Listen socket created: %d", _listenSocket);

    int opt = 1;
    if (setsockopt(_listenSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        ESP_LOGE(HAL_TCP_TAG, "setsockopt(SO_REUSEADDR) failed, errno: %d", errno);
        ::close(_listenSocket);
        _listenSocket = -1;
        return false;
    }

    // Set to non-blocking
    int flags = fcntl(_listenSocket, F_GETFL, 0);
    if (flags == -1) {
        ESP_LOGE(HAL_TCP_TAG, "fcntl(F_GETFL) failed for listen socket, errno: %d", errno);
        ::close(_listenSocket);
        _listenSocket = -1;
        return false;
    }
    if (fcntl(_listenSocket, F_SETFL, flags | O_NONBLOCK) == -1) {
        ESP_LOGE(HAL_TCP_TAG, "fcntl(F_SETFL, O_NONBLOCK) failed for listen socket, errno: %d", errno);
        ::close(_listenSocket);
        _listenSocket = -1;
        return false;
    }

    sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = ip; // INADDR_ANY or specific IP
    server_addr.sin_port = htons(port);

    if (::bind(_listenSocket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(HAL_TCP_TAG, "Bind failed for port %u, errno: %d", port, errno);
        ::close(_listenSocket);
        _listenSocket = -1;
        return false;
    }
    ESP_LOGD(HAL_TCP_TAG, "Socket bound to port %u", port);

    if (::listen(_listenSocket, MAX_ACTIVE_SOCKETS) < 0) {
        ESP_LOGE(HAL_TCP_TAG, "Listen failed, errno: %d", errno);
        ::close(_listenSocket);
        _listenSocket = -1;
        return false;
    }
    ESP_LOGI(HAL_TCP_TAG, "Server listening on port %u, socket %d", port, _listenSocket);
    return true;
}

// Basic stringToIP helper (can be expanded or use one from existing HAL if available)
static uint32_t stringToIP(const char* ip_str) {
    struct in_addr addr;
    if (inet_pton(AF_INET, ip_str, &addr) == 1) {
        return addr.s_addr;
    }
    ESP_LOGE(HAL_TCP_TAG, "Invalid IP address format: %s", ip_str);
    return 0;
}

bool TCP::setupClientSocket(const char* serverIP, uint16_t port) {
    uint32_t ip_addr = stringToIP(serverIP);
    if (ip_addr == 0) {
        return false; // Error logged in stringToIP
    }

    _clientSocket = ::socket(AF_INET, SOCK_STREAM, 0);
    if (_clientSocket < 0) {
        ESP_LOGE(HAL_TCP_TAG, "Failed to create client socket, errno: %d", errno);
        return false;
    }
    ESP_LOGD(HAL_TCP_TAG, "Client socket created: %d", _clientSocket);
    
    // Set to non-blocking for connect attempt with select
    int flags = fcntl(_clientSocket, F_GETFL, 0);
    if (flags == -1) {
         ESP_LOGE(HAL_TCP_TAG, "fcntl(F_GETFL) failed for client socket, errno: %d", errno);
        ::close(_clientSocket);
        _clientSocket = -1;
        return false;
    }
    if (fcntl(_clientSocket, F_SETFL, flags | O_NONBLOCK) == -1) {
        ESP_LOGE(HAL_TCP_TAG, "fcntl(F_SETFL, O_NONBLOCK) for client socket failed, errno: %d", errno);
        ::close(_clientSocket);
        _clientSocket = -1;
        return false;
    }

    sockaddr_in remote_addr;
    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(port);
    remote_addr.sin_addr.s_addr = ip_addr;

    ESP_LOGI(HAL_TCP_TAG, "Attempting to connect to %s:%u (socket %d)...", serverIP, port, _clientSocket);
    int ret = ::connect(_clientSocket, (struct sockaddr*)&remote_addr, sizeof(remote_addr));

    if (ret < 0) {
        if (errno == EINPROGRESS) {
            ESP_LOGD(HAL_TCP_TAG, "Connection in progress...");
            fd_set wfds;
            FD_ZERO(&wfds);
            FD_SET(_clientSocket, &wfds);

            struct timeval tv;
            tv.tv_sec = 5; // 5 second timeout for connection
            tv.tv_usec = 0;

            ret = ::select(_clientSocket + 1, nullptr, &wfds, nullptr, &tv);
            if (ret < 0) {
                ESP_LOGE(HAL_TCP_TAG, "select() for connect failed, errno: %d", errno);
                ::close(_clientSocket);
                _clientSocket = -1;
                return false;
            } else if (ret == 0) {
                ESP_LOGE(HAL_TCP_TAG, "Connect timeout to %s:%u", serverIP, port);
                ::close(_clientSocket);
                _clientSocket = -1;
                return false;
            } else {
                // Socket is writable, check SO_ERROR
                int so_error;
                socklen_t len = sizeof(so_error);
                if (getsockopt(_clientSocket, SOL_SOCKET, SO_ERROR, &so_error, &len) < 0) {
                    ESP_LOGE(HAL_TCP_TAG, "getsockopt(SO_ERROR) failed, errno: %d", errno);
                    ::close(_clientSocket);
                    _clientSocket = -1;
                    return false;
                }
                if (so_error != 0) {
                    ESP_LOGE(HAL_TCP_TAG, "Connect failed with SO_ERROR: %d", so_error);
                    ::close(_clientSocket);
                    _clientSocket = -1;
                    return false;
                }
                ESP_LOGI(HAL_TCP_TAG, "Connected to %s:%u successfully!", serverIP, port);
            }
        } else {
            ESP_LOGE(HAL_TCP_TAG, "Connect failed immediately, errno: %d", errno);
            ::close(_clientSocket);
            _clientSocket = -1;
            return false;
        }
    } else {
         // Connected immediately (should be rare for non-blocking)
         ESP_LOGI(HAL_TCP_TAG, "Connected to %s:%u immediately!", serverIP, port);
    }
    
    // Sockets used with select should generally be non-blocking.
    // The previous fcntl call already set it to O_NONBLOCK.

    // In client mode, _activeSockets will contain the _clientSocket for unified handling in tcpTask if desired,
    // or tcpTask can handle _clientSocket specially. For now, let's add it.
    // Lock guard(_socketsMutex); // Not strictly needed here as task is not running yet
    // _activeSockets.push_back(_clientSocket); 
    // Decided to handle _clientSocket separately in runTcpTask to make client logic clearer
    return true;
}


bool TCP::beginServer(uint16_t port, uint32_t ip) {
    if (_isRunning) {
        ESP_LOGW(HAL_TCP_TAG, "Server already running.");
        return false;
    }
    ESP_LOGI(HAL_TCP_TAG, "Beginning server on port %u", port);
    _isServer = true;

    if (!setupServerSocket(port, ip)) {
        return false; // Error logged in setupServerSocket
    }

    _rxQueue = xQueueCreate(RX_QUEUE_SIZE, sizeof(int));
    if (!_rxQueue) {
        ESP_LOGE(HAL_TCP_TAG, "Failed to create RX queue.");
        stop(); // Cleanup listen socket
        return false;
    }
    ESP_LOGD(HAL_TCP_TAG, "RX queue created.");

    _isRunning = true;
    BaseType_t taskCreated = xTaskCreatePinnedToCore(
        tcpTask,
        "ModbusHALtcpSrv",
        4096, // Stack size
        this, // Task parameter
        5,    // Priority
        &_tcpTaskHandle,
        tskNO_AFFINITY // Run on any core
    );

    if (taskCreated != pdPASS) {
        ESP_LOGE(HAL_TCP_TAG, "Failed to create server TCP task.");
        _tcpTaskHandle = nullptr; // Ensure it's null
        stop(); // Cleanup queue and socket
        return false;
    }
    ESP_LOGI(HAL_TCP_TAG, "Server TCP task created and started.");
    return true;
}

bool TCP::beginClient(const char* serverIP, uint16_t port) {
    if (_isRunning) {
        ESP_LOGW(HAL_TCP_TAG, "Client already running or server mode active.");
        return false;
    }
    ESP_LOGI(HAL_TCP_TAG, "Beginning client to %s:%u", serverIP, port);
    _isServer = false;

    if (!setupClientSocket(serverIP, port)) {
        return false; // Error logged in setupClientSocket
    }

    _rxQueue = xQueueCreate(RX_QUEUE_SIZE, sizeof(int));
    if (!_rxQueue) {
        ESP_LOGE(HAL_TCP_TAG, "Failed to create RX queue for client.");
        stop(); // Cleanup client socket
        return false;
    }
    ESP_LOGD(HAL_TCP_TAG, "RX queue created for client.");
    
    _isRunning = true;
    BaseType_t taskCreated = xTaskCreatePinnedToCore(
        tcpTask,
        "ModbusHALtcpCli",
        4096, // Stack size
        this, // Task parameter
        5,    // Priority
        &_tcpTaskHandle,
        tskNO_AFFINITY // Run on any core
    );

    if (taskCreated != pdPASS) {
        ESP_LOGE(HAL_TCP_TAG, "Failed to create client TCP task.");
        _tcpTaskHandle = nullptr; // Ensure it's null
        stop(); // Cleanup queue and socket
        return false;
    }
    ESP_LOGI(HAL_TCP_TAG, "Client TCP task created and started.");
    return true;
}

void TCP::tcpTask(void* param) {
    TCP* self = static_cast<TCP*>(param);
    ESP_LOGI(HAL_TCP_TAG, "tcpTask started on core %d", xPortGetCoreID());
    if (self) {
        self->runTcpTask();
    }
    ESP_LOGI(HAL_TCP_TAG, "tcpTask exiting for %s.", self->_isServer ? "server" : "client");
    // Mark task handle as null before deleting, so stop() doesn't try to use it.
    // This assumes stop() is not called concurrently with task exit.
    // A more robust mechanism might involve the task signalling its termination.
    // self->_tcpTaskHandle = nullptr; // Let stop() handle this or use a different flag for task state
    vTaskDelete(nullptr);
}

void TCP::runTcpTask() {
    // Implementation of the select loop will go here in the next step.
    // For now, just a placeholder loop.
    struct timeval tv; // For select timeout

    while (_isRunning) {
        tv.tv_sec = 0;
        tv.tv_usec = 10000; // 10ms select timeout, as per original guidelines (1ms was too short for typical FreeRTOS tick)

        fd_set readfds;
        FD_ZERO(&readfds);
        int max_fd = -1;

        { // Scope for socketsMutex lock
            Lock guard(_socketsMutex);
            if (_isServer && _listenSocket != -1) {
                FD_SET(_listenSocket, &readfds);
                max_fd = std::max(max_fd, _listenSocket);
                // ESP_LOGD(HAL_TCP_TAG, "Server: Added listen_socket %d to fd_set", _listenSocket);
            } else if (!_isServer && _clientSocket != -1) {
                FD_SET(_clientSocket, &readfds);
                max_fd = std::max(max_fd, _clientSocket);
                // ESP_LOGD(HAL_TCP_TAG, "Client: Added client_socket %d to fd_set", _clientSocket);
            }

            for (int sock : _activeSockets) { // Primarily for server mode client connections
                FD_SET(sock, &readfds);
                max_fd = std::max(max_fd, sock);
                // ESP_LOGD(HAL_TCP_TAG, "Server: Added active_socket %d to fd_set", sock);
            }
        } // Unlock socketsMutex

        if (max_fd == -1) { // No sockets to monitor (e.g., client not connected yet, or server not started)
            vTaskDelay(pdMS_TO_TICKS(20)); // Sleep a bit
            continue;
        }
        
        // ESP_LOGD(HAL_TCP_TAG, "Calling select() with max_fd = %d", max_fd);
        int activity = ::select(max_fd + 1, &readfds, nullptr, nullptr, &tv);

        if (!_isRunning) break; // Check flag again after select

        if (activity < 0) {
            if (errno == EINTR) { // Interrupted system call
                ESP_LOGD(HAL_TCP_TAG, "select() interrupted, retrying.");
                continue;
            }
            ESP_LOGE(HAL_TCP_TAG, "select() error, errno: %d. Task stopping.", errno);
            _isRunning = false; // Stop the task loop on critical select error
            break;
        }

        if (activity == 0) {
            // Timeout, just loop again
            // ESP_LOGD(HAL_TCP_TAG, "select() timeout");
            continue;
        }

        // Activity detected, process sockets
        { // Scope for socketsMutex lock
            Lock guard(_socketsMutex);
            if (!_isRunning) break; // Check flag again after acquiring lock

            // 1. Handle new connections for server
            if (_isServer && _listenSocket != -1 && FD_ISSET(_listenSocket, &readfds)) {
                ESP_LOGD(HAL_TCP_TAG, "Activity on listen socket %d", _listenSocket);
                sockaddr_in client_addr;
                socklen_t client_len = sizeof(client_addr);
                int new_socket = ::accept(_listenSocket, (struct sockaddr*)&client_addr, &client_len);

                if (new_socket >= 0) {
                    if (_activeSockets.size() < MAX_ACTIVE_SOCKETS) {
                        // Set new socket to non-blocking
                        int flags = fcntl(new_socket, F_GETFL, 0);
                        if (flags != -1 && fcntl(new_socket, F_SETFL, flags | O_NONBLOCK) != -1) {
                            _activeSockets.push_back(new_socket);
                            char client_ip[INET_ADDRSTRLEN];
                            inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
                            ESP_LOGI(HAL_TCP_TAG, "New connection from %s on socket %d. Active clients: %d", client_ip, new_socket, _activeSockets.size());
                        } else {
                            ESP_LOGE(HAL_TCP_TAG, "Failed to set new client socket %d to non-blocking, errno: %d. Closing.", new_socket, errno);
                            ::close(new_socket);
                        }
                    } else {
                        ESP_LOGW(HAL_TCP_TAG, "Max clients (%d) reached. Rejecting new connection from socket %d.", MAX_ACTIVE_SOCKETS, new_socket);
                        ::close(new_socket);
                    }
                } else {
                    if (errno != EAGAIN && errno != EWOULDBLOCK) {
                         ESP_LOGE(HAL_TCP_TAG, "accept() failed on listen socket %d, errno: %d", _listenSocket, errno);
                         // Potentially a serious issue with the listen socket, consider re-init or stopping.
                    }
                }
            }

            // 2. Signaler simplement quels sockets ont des données à lire
            std::vector<int> sockets_to_check = _activeSockets;
            if (!_isServer && _clientSocket != -1) {
                sockets_to_check.clear();
                sockets_to_check.push_back(_clientSocket);
            }

            std::vector<int> sockets_to_remove;

            uint8_t dummy;
            for (int sock : sockets_to_check) {
                if (!FD_ISSET(sock, &readfds)) continue;

                ssize_t peek = ::recv(sock, &dummy, 1, MSG_PEEK);

                if (peek > 0) {
                    if (xQueueSend(_rxQueue, &sock, 0) != pdTRUE) {
                        ESP_LOGW(HAL_TCP_TAG, "RX queue full. Dropping event for socket %d.", sock);
                    }
                } else if (peek == 0) {
                    ESP_LOGI(HAL_TCP_TAG, "Socket %d closed by peer.", sock);
                    sockets_to_remove.push_back(sock);
                } else {
                    if (errno != EAGAIN && errno != EWOULDBLOCK) {
                        ESP_LOGE(HAL_TCP_TAG, "recv(MSG_PEEK) error on socket %d, errno: %d. Closing.", sock, errno);
                        sockets_to_remove.push_back(sock);
                    }
                }
            }

            for (int s : sockets_to_remove) {
                closeSocket(s);
            }

        } // Unlock socketsMutex
    } // while (_isRunning)
}

// Private helper to close a socket and remove it from tracking
void TCP::closeSocket(int sock) {
    // Assumes _socketsMutex is held by caller if necessary, or called during stop()
    // For runTcpTask, it should be called within the _socketsMutex lock.
    ESP_LOGD(HAL_TCP_TAG, "Closing socket %d.", sock);
    ::close(sock);
    if (_isServer) {
        auto it = std::remove(_activeSockets.begin(), _activeSockets.end(), sock);
        if (it != _activeSockets.end()) {
            _activeSockets.erase(it, _activeSockets.end());
            ESP_LOGI(HAL_TCP_TAG, "Removed socket %d from active server clients. Count: %d", sock, _activeSockets.size());
        }
    } else { // Client mode
        if (sock == _clientSocket) {
            _clientSocket = -1;
            // Optionally, attempt reconnect or notify higher layer
            ESP_LOGI(HAL_TCP_TAG, "Client socket %d disconnected.", sock);
        }
    }
}


// Implementation for sendMsg, getNextMsg, getActiveSocketCount, etc. will follow.
bool TCP::sendMsg(const uint8_t* payload, const size_t len, const int destSocket, int* actualSocket) {
    // To be implemented
    if (!_isRunning) return false;

    int targetSocket = -1;
    if (destSocket != -1) { // Explicit destination
        targetSocket = destSocket;
    } else { // Default destination
        Lock guard(_socketsMutex); // Protect access to _clientSocket or _activeSockets
        if (!_isServer) { // Client mode
            targetSocket = _clientSocket;
        } else { // Server mode, destSocket = -1 (or 0 from original guide) means "last active"
                 // This HAL version uses msg.socketNum from incoming message for replies.
                 // If sending unsolicited or broadcast, needs more logic.
                 // For now, this case needs clarification if destSocket is truly -1 for server.
                 // The guideline was: destSocket = 0 → envoie au dernier client actif.
                 // Let's assume destSocket in msg will be the reply target for server.
                 // If this sendMsg is for a server reply, msg.socketNum should be set by ModbusTCP layer.
            ESP_LOGE(HAL_TCP_TAG, "sendMsg: Server mode with default destSocket (-1) not fully supported without target.");
            return false; // Or use a "last active" logic if implemented.
        }
    }
    
    if (actualSocket) {
        *actualSocket = targetSocket;
    }

    if (targetSocket == -1) {
        ESP_LOGE(HAL_TCP_TAG, "sendMsg: No valid target socket.");
        return false;
    }

    if (len == 0 || len > MAX_MODBUS_FRAME_SIZE) {
        ESP_LOGE(HAL_TCP_TAG, "sendMsg: Invalid message length %d.", len);
        return false;
    }
    
    ESP_LOGD(HAL_TCP_TAG, "Sending %d bytes to socket %d", len, targetSocket);
    ssize_t sent_bytes = ::send(targetSocket, payload, len, 0);

    if (sent_bytes < 0) {
        ESP_LOGE(HAL_TCP_TAG, "send() to socket %d failed, errno: %d", targetSocket, errno);
        // Consider closing socket on certain errors like EPIPE, ECONNRESET
        if (errno == EPIPE || errno == ECONNRESET || errno == ENOTCONN || errno == EBADF) {
            Lock guard(_socketsMutex);
            closeSocket(targetSocket);
        }
        return false;
    }
    if ((size_t)sent_bytes != len) {
        ESP_LOGW(HAL_TCP_TAG, "send() on socket %d: Incomplete send. Sent %d of %d bytes.", targetSocket, sent_bytes, len);
        return false; // Or handle partial sends if appropriate
    }

    ESP_LOGD(HAL_TCP_TAG, "Successfully sent %d bytes to socket %d", sent_bytes, targetSocket);
    return true;
}


size_t TCP::getActiveSocketCount() {
    Lock guard(_socketsMutex);
    if (_isServer) {
        return _activeSockets.size();
    } else {
        return (_clientSocket != -1) ? 1 : 0;
    }
}

bool TCP::isServerRunning() const {
    return _isRunning && _isServer && _listenSocket != -1;
}

bool TCP::isClientConnected() {
    Lock guard(_socketsMutex);
    return _isRunning && !_isServer && _clientSocket != -1;
}

// Server ctor
TCP::TCP(uint16_t serverPort) : TCP() {
    _cfgMode = CfgMode::SERVER;
    _cfgPort = serverPort;
}

// Client ctor
TCP::TCP(const char* serverIP, uint16_t port) : TCP() {
    _cfgMode = CfgMode::CLIENT;
    _cfgIP = serverIP ? serverIP : "";
    _cfgPort = port;
}

bool TCP::begin() {
    switch (_cfgMode) {
        case CfgMode::SERVER:
            return beginServer(_cfgPort);
        case CfgMode::CLIENT:
            return beginClient(_cfgIP.c_str(), _cfgPort);
        default:
            return false; // No config
    }
}

// =======================================================================
// Lecture non bloquante d'un socket (API utilisée par ModbusInterface::TCP)
// =======================================================================

size_t TCP::readSocketData(int socketNum, uint8_t* dst, size_t maxLen) {
    if (!dst || maxLen == 0) return SIZE_MAX;

    ssize_t r = ::recv(socketNum, dst, maxLen, 0); // Non-bloquant (socket configuré O_NONBLOCK)

    if (r > 0) {
        return static_cast<size_t>(r);
    }

    if (r == 0) {
        ESP_LOGI(HAL_TCP_TAG, "readSocketData: socket %d closed by peer", socketNum);
        return SIZE_MAX;
    }

    if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return 0; // Pas de data prête.
    }

    ESP_LOGE(HAL_TCP_TAG, "readSocketData: recv error on socket %d, errno %d", socketNum, errno);
    return SIZE_MAX;
}

} // namespace ModbusHAL