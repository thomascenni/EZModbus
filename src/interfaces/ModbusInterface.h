/**
 * @file ModbusInterface.h
 * @brief Modbus interface abstract class
 */

#pragma once

#include "core/ModbusCore.h"
#include "core/ModbusCodec.h"
#include "utils/ModbusDebug.h"

namespace ModbusInterface {

    class IInterface;

    /* @brief Receive callback type
     * @param frame The frame to process
     * @note Validity of the frame is guaranteed ONLY until the callback returns
     *       The application class is responsible for making a local copy of the frame
     *       if needed
     */
    using RcvCallback = std::function<void(const Modbus::Frame& frame)>;


class IInterface {
public:

    // Define max number of registered callbacks stored
    // (number of instances of applications like ModbusClient & ModbusServer
    // that can share the same interface)
    static constexpr uint8_t MAX_RCV_CALLBACKS = 5; 

    // Result of a Modbus operation
    enum Result {
        SUCCESS,
        NODATA,
        ERR_INIT_FAILED,
        ERR_INVALID_FRAME,
        ERR_BUSY,
        ERR_RX_FAILED,
        ERR_SEND_FAILED,
        ERR_INVALID_MSG_TYPE,
        ERR_INVALID_TRANSACTION_ID,
        ERR_TIMEOUT,
        ERR_INVALID_ROLE,
        ERR_ADD_CALLBACK_BUSY,
        ERR_TOO_MANY_CALLBACKS,
        ERR_NO_CALLBACKS,
        ERR_NOT_INITIALIZED,
        ERR_CONNECTION_FAILED,
        ERR_CONFIG_FAILED
    };
    static constexpr const char* toString(Result result) {
        switch (result) {
            case SUCCESS: return "success";
            case NODATA: return "no data to process";
            case ERR_INIT_FAILED: return "init failed";
            case ERR_INVALID_FRAME: return "invalid frame";
            case ERR_BUSY: return "busy";
            case ERR_RX_FAILED: return "RX error";
            case ERR_SEND_FAILED: return "send failed";
            case ERR_INVALID_MSG_TYPE: return "invalid message type";
            case ERR_INVALID_TRANSACTION_ID: return "transaction id mismatch";
            case ERR_TIMEOUT: return "timeout";
            case ERR_INVALID_ROLE: return "invalid role";
            case ERR_ADD_CALLBACK_BUSY: return "callback store is busy";
            case ERR_TOO_MANY_CALLBACKS: return "too many callbacks stored";
            case ERR_NO_CALLBACKS: return "no callbacks stored";
            case ERR_NOT_INITIALIZED: return "interface not initialized";
            case ERR_CONNECTION_FAILED: return "connection failed";
            case ERR_CONFIG_FAILED: return "configuration failed";
            default: return "unknown error";
        }
    }

    // Helper to cast an error
    // - Returns a Result
    // - Captures point of call context & prints a log message when debug 
    // is enabled. No overhead when debug is disabled (except for
    // the desc string, if any)
    static inline Result Error(Result res, const char* desc = nullptr
                        #ifdef EZMODBUS_DEBUG
                        , const char* fileName = __builtin_FILE()
                        , const char* funcName = __builtin_FUNCTION() 
                        , int lineNo = __builtin_LINE()
                        #endif
                        ) {
        #ifdef EZMODBUS_DEBUG
            std::string logMessage = std::string("Error: ") + toString(res);
            if (desc && *desc != '\0') {
                logMessage += std::string(" (") + desc + ")";
            }
            Modbus::Debug::LOG_MSG(logMessage, fileName, funcName, lineNo);
        #endif
        return res;
    }

    // Helper to cast a success
    // - Returns Result::SUCCESS
    // - Captures point of call context & prints a log message when debug 
    // is enabled. No overhead when debug is disabled (except for
    // the desc string, if any)
    static inline Result Success(const char* desc = nullptr
                          #ifdef EZMODBUS_DEBUG
                          , const char* fileName = __builtin_FILE()
                          , const char* funcName = __builtin_FUNCTION() 
                          , int lineNo = __builtin_LINE()
                          #endif
                          ) {
        #ifdef EZMODBUS_DEBUG
            if (desc && *desc != '\0') {
                std::string logMessage = std::string("Success: ") + desc;
                Modbus::Debug::LOG_MSG(logMessage, fileName, funcName, lineNo);
            }
        #endif
        return SUCCESS;
    }


    virtual ~IInterface() = default;
    
    /* @brief Initialize the interface
     * @return The result of the operation
     */
    virtual Result begin() = 0;

    /* @brief Send a frame, with optional callback and/or tracker
     * @param frame The frame to send
     * @param callback The callback to call when the response is received
     * @param tracker The tracker to follow up the status of the request by the caller
     * @return The result of the operation
     */
    virtual Result sendFrame(const Modbus::Frame& frame, TaskHandle_t notifyTask = nullptr) = 0;

    /* @brief Check if the interface is ready to transmit a frame
     * @return true if the interface is ready, false otherwise
     */
    virtual bool isReady() = 0;

    /* @brief Get the role of the interface
     * @return The role of the interface
     */
    Modbus::Role getRole() const {
        return _role;
    }

    /* @brief Set a callback for spontaneous messages
     * @param callback The callback to call when a message is received
     * @return The result of the operation
     */
    Result setRcvCallback(RcvCallback callback) {
        Lock guard(_callbackMutex, 0);  // Try-lock sans attente
        if (!guard.isLocked()) return Error(ERR_ADD_CALLBACK_BUSY);

        for (auto& cb : _rcvCallbacks) {
            if (!cb) {
                cb = callback;
                return Success();
            }
        }
        return Error(ERR_TOO_MANY_CALLBACKS);
    }

    /* @brief Notify all callbacks for a spontaneous message
     * @param frame The frame to notify
     * @return The result of the operation
     */
    Result notifyCallbacks(const Modbus::Frame& frame) {
        Lock guard(_callbackMutex);  // Lock avec attente infinie car critique
        bool hasCallback = false;
        for (const auto& callback : _rcvCallbacks) {
            if (callback) {
                hasCallback = true;
                callback(frame);
                Modbus::Debug::LOG_MSG("Callback notified");
            }
        }
        return hasCallback ? Success() : Error(ERR_NO_CALLBACKS);
    }

    /* @brief Get the catch all slave IDs flag
     * @return true if the catch all slave IDs flag is set, false otherwise
     */
    inline bool checkCatchAllSlaveIds() const {
        return _catchAllSlaveIds;
    }

protected:

    // Callback for spontaneous messages & lock to protect against concurrent access
    std::array<RcvCallback, MAX_RCV_CALLBACKS> _rcvCallbacks;
    Mutex _callbackMutex; 
    Modbus::Role _role;
    bool _catchAllSlaveIds = false; // Can be overridden by the derived class to ignore Slave ID field (especially TCP server)

}; // class IInterface

} // namespace ModbusInterface
