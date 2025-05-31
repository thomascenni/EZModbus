/**
 * @file ModbusTCP.h
 * @brief Modbus TCP class header
 */

#pragma once

#include "core/ModbusCore.h"
#include "core/ModbusCodec.h"
#include "interfaces/ModbusInterface.h"
#include "utils/ModbusDebug.h"
#include "drivers/ModbusHAL_TCP.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "apps/ModbusClient.h" // To get the default request timeout

namespace ModbusInterface {

class TCP : public IInterface {
public:

    // ===================================================================================
    // CONSTANTS
    // ===================================================================================

    static constexpr uint32_t RX_ASSEMBLY_TIMEOUT_MS = 50;
    static constexpr uint32_t RXTX_QUEUE_CHECK_TIMEOUT_MS = 100; // Added for task management
    // Transaction timeout : take client txn timeout - 50 ms
    static constexpr uint32_t TCP_TRANSACTION_TIMEOUT_MS = std::max((uint32_t)50, 
                                                                Modbus::Client::DEFAULT_REQUEST_TIMEOUT_MS - (uint32_t)50);

    // Tasks stack sizes (higher for debug to let room for the printf/hexdump buffers)
    #ifdef EZMODBUS_DEBUG
        static constexpr uint32_t RXTX_TASK_STACK_SIZE = 6144;
    #else
        static constexpr uint32_t RXTX_TASK_STACK_SIZE = 4096;
    #endif

    // ===================================================================================
    // DATA STRUCTURES
    // ===================================================================================

    /* @brief Stores the current transaction context */
    struct TransactionCtx {
        bool active = false;
        int startMs = 0;
        int socketNum = -1;
        uint16_t tid = 0;

        void set(int socketNum, uint16_t tid) { active=true; startMs=TIME_MS(); this->socketNum=socketNum; this->tid=tid; }
        void clear() { active=false; startMs=0; socketNum=-1; tid=0; }
    };

    /* @brief Stores the current transmission data context
    * @note This structs only holds the metadata of the current transmission, not the data itself
    *       which is stored in _txBuffer
    */
    struct TxCtx {
        uint16_t tid = 0;
        int destSock = -1;
        bool isBroadcast = false;
        TaskHandle_t notifyTask = nullptr;

        void set(uint16_t tid, int socketNum, bool isBroadcast = false, TaskHandle_t notifyTask = nullptr) { 
            this->tid = tid; 
            this->destSock = socketNum; 
            this->isBroadcast = isBroadcast; 
            this->notifyTask = notifyTask; 
        }
        void clear() { tid=0; destSock=-1; isBroadcast=false; notifyTask=nullptr; }
    };

    // ===================================================================================
    // CONSTRUCTOR & PUBLIC METHODS
    // ===================================================================================

    explicit TCP(ModbusHAL::TCP& hal, Modbus::Role role);
    virtual ~TCP();

    Result begin() override;
    Result sendFrame(const Modbus::Frame &frame, TaskHandle_t notifyTask = nullptr) override;
    bool isReady() override;
    TaskHandle_t getRxTxTaskHandle();

private:
    // ===================================================================================
    // PRIVATE MEMBERS
    // ===================================================================================

    // TCP communication
    ModbusHAL::TCP& _tcpHAL;
    bool _isInitialized = false; // Added initialization flag

    // Data buffers & protection
    std::array<uint8_t, ModbusCodec::TCP::MAX_FRAME_SIZE> _rxBuf;
    ByteBuffer _rxBuffer; // Linked to _rxBuf
    std::array<uint8_t, ModbusCodec::TCP::MAX_FRAME_SIZE> _txBuf;
    ByteBuffer _txBuffer; // Linked to _txBuf
    TxCtx _txCtx; // Stores the current TX data context
    Mutex _txMutex; // Protects access to _txBuffer & _txCtx

    // Transaction management
    TransactionCtx _currentTransaction;
    Mutex _transactionMutex;

    // Data processing
    QueueHandle_t _rxEventQueue = nullptr;
    QueueHandle_t _txRequestQueue = nullptr;
    QueueSetHandle_t _eventQueueSet = nullptr;

    // ===================================================================================
    // PRIVATE METHODS
    // ===================================================================================

    // Lifecycle management
    void beginCleanup();
    void killRxTxTask();
    
    // RX frame processing
    Result processReceivedFrame(const ByteBuffer& frameBytes, int socketNum);
    Result handleDecodedFrame(const Modbus::Frame& frame, int socketNum, uint16_t transactionId);
    Result fetchSocketData(int socketNum, ByteBuffer& rcvBuf, uint16_t& rcvTid);

    // TX frame processing
    Result handleTxRequest();
    inline void notifyTaskWithResult(TaskHandle_t t, Result res);

    // Transaction management
    uint16_t getNextOutgoingTransactionId();
    bool beginTransaction(int socketNum, uint16_t transactionId);
    void endTransaction();

    // ===================================================================================
    // TASKS
    // ===================================================================================

    static void rxTxTask(void* tcp);
    TaskHandle_t _rxTxTaskHandle;

};

} // namespace ModbusInterface