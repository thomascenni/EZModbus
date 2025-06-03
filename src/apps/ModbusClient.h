/**
 * @file ModbusClient.h
 * @brief Modbus client class header
 */

#pragma once

#include "core/ModbusCore.h"
#include "interfaces/ModbusInterface.h"
#include "utils/ModbusDebug.h"


namespace Modbus {

class Client {
public:
    // ===================================================================================
    // CONSTANTS
    // ===================================================================================

    static constexpr uint32_t DEFAULT_REQUEST_TIMEOUT_MS = 1000; // Max RTT before aborting the current request

    // Tasks stack sizes (higher for debug to let room for the printf/hexdump buffers)
    #ifdef EZMODBUS_DEBUG
        static constexpr uint32_t TX_RESULT_TASK_STACK_SIZE = 4096;
    #else
        static constexpr uint32_t TX_RESULT_TASK_STACK_SIZE = 2048;
    #endif

    // ===================================================================================
    // RESULT TYPES
    // ===================================================================================

    enum Result {
        SUCCESS,
        NODATA,
        ERR_INVALID_FRAME,
        ERR_BUSY,
        ERR_TX_FAILED,
        ERR_TIMEOUT,
        ERR_INVALID_RESPONSE,
        ERR_NOT_INITIALIZED,
        ERR_INIT_FAILED
    };
    static constexpr const char* toString(const Result result) {
        switch (result) {
            case SUCCESS: return "success";
            case NODATA: return "no data (ongoing transaction)";
            case ERR_INVALID_FRAME: return "invalid frame";
            case ERR_BUSY: return "busy";
            case ERR_TX_FAILED: return "tx failed";
            case ERR_TIMEOUT: return "timeout";
            case ERR_INVALID_RESPONSE: return "invalid response";
            case ERR_NOT_INITIALIZED: return "client not initialized";
            case ERR_INIT_FAILED: return "init failed";
            default: return "unknown result";
        }
    }

    /* @brief Helper to cast an error
     * @return The error result
     * @note Captures point of call context & prints a log message when debug 
     * is enabled. No overhead when debug is disabled (except for
     * the desc string, if any)
     */
    static inline Result Error(Result res, const char* desc = nullptr
                        #ifdef EZMODBUS_DEBUG
                        , Modbus::Debug::CallCtx ctx = Modbus::Debug::CallCtx()
                        #endif
                        ) {
        #ifdef EZMODBUS_DEBUG
            std::string logMessage = std::string("Error: ") + toString(res);
            if (desc && *desc != '\0') {
                logMessage += std::string(" (") + desc + ")";
            }
            Modbus::Debug::LOG_MSG(logMessage, ctx);
        #endif
        return res;
    }

    /* @brief Helper to cast a success
     * @return Result::SUCCESS
     * @note Captures point of call context & prints a log message when debug 
     * is enabled. No overhead when debug is disabled (except for
     * the desc string, if any)
     */
    static inline Result Success(const char* desc = nullptr
                          #ifdef EZMODBUS_DEBUG
                          , Modbus::Debug::CallCtx ctx = Modbus::Debug::CallCtx()
                          #endif
                          ) {
        #ifdef EZMODBUS_DEBUG
            if (desc && *desc != '\0') {
                std::string logMessage = std::string("Success: ") + desc;
                Modbus::Debug::LOG_MSG(logMessage, ctx);
            }
        #endif
        return SUCCESS;
    }

    // ===================================================================================
    // CONSTRUCTOR & PUBLIC METHODS
    // ===================================================================================

    Client(ModbusInterface::IInterface& interface, uint32_t timeoutMs = DEFAULT_REQUEST_TIMEOUT_MS);
    ~Client();

    Result begin();
    Result sendRequest(const Modbus::Frame& request, 
                      Modbus::Frame& response,
                      Result* userTracker = nullptr);
    bool isReady();

private:
    // ===================================================================================
    // PRIVATE DATA STRUCTURES
    // ===================================================================================
    
    /* @brief Encapsulates the management of a pending request lifecycle with a thread-safe API
     * @note PendingRequest can only be set() if inactive (clear() must be called first)
     */
    class PendingRequest {
    private:
        Modbus::Frame _reqMetadata; // Does not store request data (only fc, slaveId, regAddress, regCount)
        Modbus::Frame* _pResponse = nullptr;
        Result* _tracker = nullptr;
        uint32_t _timestampMs = 0;
        volatile bool _active = false;
        Mutex _mutex;
        TimerHandle_t _timeoutTimer = nullptr;

    public:
        // Helper methods
        bool set(const Modbus::Frame& request, Modbus::Frame* response, Result* tracker, uint32_t timeoutMs);
        void clear();
        // Lock-free methods
        const bool isActive() const;
        const bool hasResponse() const;
        const uint32_t getTimestampMs() const;
        const Modbus::Frame& getRequestMetadata() const;
        // Locked methods
        void setResult(Result result);
        void setResponse(const Modbus::Frame& response);
        // Timeout callback
        static void timeoutCallback(TimerHandle_t timer);
        // Destructor
        ~PendingRequest();
    };

    // ===================================================================================
    // PRIVATE MEMBERS
    // ===================================================================================

    ModbusInterface::IInterface& _interface;
    uint32_t _requestTimeoutMs;
    PendingRequest _pendingRequest;
    Modbus::Frame _responseBuffer;
    bool _isInitialized = false;

    // ===================================================================================
    // PRIVATE METHODS
    // ===================================================================================

    void handleResponse(const Modbus::Frame& response);
    inline bool waitTaskNotification(uint32_t& value, const uint32_t timeoutMs = 200);

    // ===================================================================================
    // TASKS
    // ===================================================================================

    static void handleTxResultTask(void* client);
    TaskHandle_t _handleTxResultTaskHandle;

};

} // namespace Modbus