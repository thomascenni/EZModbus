/**
 * @file ModbusServer.h
 * @brief Modbus server class header
 */

#pragma once

#include "core/ModbusCore.h"
#include "interfaces/ModbusInterface.h"
#include "utils/ModbusDebug.h"

namespace Modbus {

class Server {
public:
    // ===================================================================================
    // CONSTANTS
    // ===================================================================================

    static constexpr uint32_t MAX_REGISTERS = 65535;
    static constexpr uint32_t RSV_REGISTERS = 100;
    static constexpr uint32_t RESPONSE_TX_TIMEOUT_MS = 1000;

    // ===================================================================================
    // RESULT TYPES
    // ===================================================================================

    enum Result {
        SUCCESS,
        NODATA,
        ERR_REG_BUSY,
        ERR_REG_OVERFLOW,
        ERR_REG_INVALID_TYPE,
        ERR_REG_MISSING_READCB,
        ERR_REG_MISSING_WRITECB,
        ERR_REG_READONLY_WRITECB,
        ERR_REG_NOT_FOUND,
        ERR_RCV_BUSY,
        ERR_RCV_INVALID_TYPE,
        ERR_RCV_WRONG_SLAVE_ID,
        ERR_RCV_ILLEGAL_FUNCTION,
        ERR_RCV_ILLEGAL_DATA_ADDRESS,
        ERR_RCV_ILLEGAL_DATA_VALUE,
        ERR_RCV_SLAVE_DEVICE_FAILURE,
        ERR_RSP_TX_FAILED,
        ERR_NOT_INITIALIZED,
        ERR_INIT_FAILED
    };
    static const char* toString(const Result res) {
        switch (res) {
            case SUCCESS: return "success";
            case NODATA: return "no data";
            case ERR_REG_BUSY: return "busy register store";
            case ERR_REG_OVERFLOW: return "stored too many registers";
            case ERR_REG_INVALID_TYPE: return "invalid register type";
            case ERR_REG_MISSING_READCB: return "missing read callback";
            case ERR_REG_MISSING_WRITECB: return "missing write callback";
            case ERR_REG_READONLY_WRITECB: return "readonly register w/ write callback";
            case ERR_RCV_BUSY: return "incoming request: busy";
            case ERR_RCV_INVALID_TYPE: return "received invalid request type";
            case ERR_RCV_WRONG_SLAVE_ID: return "wrong slave ID";
            case ERR_RCV_ILLEGAL_FUNCTION: return "illegal function";
            case ERR_RCV_ILLEGAL_DATA_ADDRESS: return "illegal data address";
            case ERR_RCV_ILLEGAL_DATA_VALUE: return "illegal data value";
            case ERR_RCV_SLAVE_DEVICE_FAILURE: return "slave device failure";
            case ERR_RSP_TX_FAILED: return "transmit failed";
            case ERR_NOT_INITIALIZED: return "server not initialized";
            case ERR_INIT_FAILED: return "init failed";
            default: return "unknown error";
        }
    }

    static inline Result Error(Result res, const char* desc = nullptr, 
                Modbus::Debug::CallCtx ctx = Modbus::Debug::CallCtx()) {
        std::string logMessage = std::string("Error: ") + toString(res);
        if (desc && *desc != '\0') {
            logMessage += std::string(" (") + desc + ")";
        }
        Modbus::Debug::LOG_MSG(logMessage, ctx);
        return res;
    }

    static inline Result Success(const char* desc = nullptr,
                  Modbus::Debug::CallCtx ctx = Modbus::Debug::CallCtx()) {
        if (desc && *desc != '\0') {
            std::string logMessage = std::string("Success: ") + desc;
            Modbus::Debug::LOG_MSG(logMessage, ctx);
        }
        return SUCCESS;
    }

    // ===================================================================================
    // DATA STRUCTURES
    // ===================================================================================

    struct Register; // Forward declaration

    // Callback types
    // READ (lambda): takes the register context as argument and returns the value
    using ReadCallback = std::function<uint16_t(const Register&)>;
    // WRITE (lambda): takes the value & register context as arguments and returns a bool (write success/failure)
    using WriteCallback = std::function<bool(uint16_t, const Register&)>;
    // DIRECT ACCESS: simple function pointers for direct access to the value pointer (static allocation)
    using ReadCallbackC = uint16_t (*)(const Server::Register& r);
    using WriteCallbackC = bool (*)(uint16_t value, const Server::Register& r);

    /* @brief Public API structure to define a register
     * @note If direct access pointer is defined, it will be used in priority
     */
    struct Register {
        // Register metadata
        Modbus::RegisterType type = Modbus::NULL_RT;   // Type of register
        uint16_t address = 0;                          // Register address
        const char* name = "";                         // Register name (for debugging/logging)

        // Register value (2 options) - if direct access pointer is defined, it will be used in priority
        // 1. Direct value access
        volatile uint32_t* value = nullptr;            // Pointer to the register value - uint32_t for atomicity
        // 2. Read/write callbacks
        ReadCallback readCb = nullptr;                 // Read callback
        WriteCallback writeCb = nullptr;               // Write callback

        operator bool() const {
            return Modbus::isValid(type);             // A register is valid if its type is valid
        }
    };

    // ===================================================================================
    // CONSTRUCTOR & PUBLIC METHODS
    // ===================================================================================

    Server(ModbusInterface::IInterface& interface, uint8_t slaveId = 1, bool rejectUndefined = true);
    ~Server();

    Result begin();
    Result setRegisterCount(const Modbus::RegisterType type, const uint16_t count);
    Result addRegister(const Register& reg);
    Result addRegisters(const std::vector<Register>& registers);
    Result clearAllRegisters();
    Register getRegister(Modbus::RegisterType type, uint16_t address);
    void poll();
    bool isBusy();

private:
    // ===================================================================================
    // PRIVATE DATA STRUCTURES
    // ===================================================================================

    /* @brief Internal structure to store register metadata & value pointers
     */
    struct RegisterEntry {
        Modbus::RegisterType type;
        uint16_t address;
        const char* name = "";
        volatile uint32_t* value = nullptr;
        uint32_t cbIndex = UINT32_MAX;

        Register toRegister(Server* srv);
    };

    /* @brief Internal structure to store read/write callbacks (memory optimized)
     */
    struct CBEntry {
        ReadCallback readCb{nullptr};
        WriteCallback writeCb{nullptr};
    };

    using RegisterStore = std::vector<RegisterEntry>;   // Stores a set of RegisterEntry objects
    using CBStore = std::vector<CBEntry>;               // Stores a set of CBEntry objects

    // ===================================================================================
    // PRIVATE MEMBERS
    // ===================================================================================

    ModbusInterface::IInterface& _interface;
    uint8_t _serverId;
    bool _rejectUndefined; // If false, undefined registers will be silently ignored (no exception returned)
    bool _isInitialized = false;
    
    // Mutex protection
    Mutex _handleRequestMutex;
    Mutex _registerStoreMutex;

    // Registers & callback stores
    RegisterStore _discreteInputStore;
    RegisterStore _coilStore;
    RegisterStore _inputRegisterStore;
    RegisterStore _holdingRegisterStore;
    CBStore _cbStore;

    // Callbacks for direct access to value pointers
    ReadCallbackC _directReadCb;
    WriteCallbackC _directWriteCb;

    // ===================================================================================
    // PRIVATE METHODS
    // ===================================================================================

    // Request handlers
    Result handleRequest(const Modbus::Frame& request);
    Result handleRead(const Modbus::Frame& request, Modbus::Frame& response, RegisterStore* regStore);
    Result handleWrite(const Modbus::Frame& request, Modbus::Frame& response, RegisterStore* regStore);
    Result sendResponse(const Modbus::Frame& response);

    // Find entry/store helpers
    RegisterStore* findRegisterStore(Modbus::RegisterType type);
    RegisterStore* findRegisterStore(Modbus::FunctionCode fc);
    RegisterEntry* findRegisterEntry(const Modbus::RegisterType type, const uint16_t address);
    RegisterEntry* findRegisterEntry(RegisterStore* regStore, const uint16_t address);
    CBEntry* findCBEntry(const uint32_t cbIndex);
    ReadCallback findReadCallback(const RegisterEntry& reg);
    WriteCallback findWriteCallback(const RegisterEntry& reg);
    
    // Register helpers
    bool registerExists(const Register& reg);
    bool wouldOverflow(const Modbus::RegisterType type, const size_t additionalCount);
    void addRegisterInternal(const Register& reg);
    Result isValidEntry(const Register& reg);
    static bool isWrite(const Modbus::FunctionCode fc);
    static bool isReadOnly(const Modbus::RegisterType type);
};

} // namespace Modbus 