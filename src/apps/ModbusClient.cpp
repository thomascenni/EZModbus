/**
 * @file ModbusClient.cpp
 * @brief Modbus client implementation
 */

#include "ModbusClient.h"

namespace Modbus {

// ===================================================================================
// DATA STRUCTURES METHODS
// ===================================================================================

/* @brief Timeout callback for pending request
 * @param timer The timer handle
 */
void Client::PendingRequest::timeoutCallback(TimerHandle_t timer) {
    auto* pendingReq = static_cast<PendingRequest*>(pvTimerGetTimerID(timer));
    if (pendingReq && pendingReq->isActive()) {
        pendingReq->setResult(ERR_TIMEOUT);
        pendingReq->clear();
        Modbus::Debug::LOG_MSG("Request timed out via timer");
    }
}

/* @brief Set the pending request
 * @param request The request to set
 * @param response Where to store the response (response.data will be resized to match its size)
 * @param tracker Pointer to the transfer result tracker
 * @param timeoutMs Timeout in milliseconds
 * @return true if the pending request was set successfully, false is already active (clear it first)
 */
bool Client::PendingRequest::set(const Modbus::Frame& request, Modbus::Frame* response, Result* tracker, uint32_t timeoutMs) {
    if (isActive()) return false; // The pending request must be cleared first
    Lock guard(_mutex);
    _reqMetadata.fc = request.fc;
    _reqMetadata.slaveId = request.slaveId;
    _reqMetadata.regAddress = request.regAddress;
    _reqMetadata.regCount = request.regCount;
    _pResponse = response;
    _tracker = tracker;
    _timestampMs = TIME_MS();
    _active = true;
    
    // Create and start timeout timer
    if (!_timeoutTimer) {
        _timeoutTimer = xTimerCreate(
            "ModbusTimeout",
            pdMS_TO_TICKS(timeoutMs),
            pdFALSE,  // one-shot
            this,     // timer ID = this PendingRequest
            timeoutCallback
        );
    } else {
        // Update timer period and restart
        xTimerChangePeriod(_timeoutTimer, pdMS_TO_TICKS(timeoutMs), 0);
    }
    
    if (_timeoutTimer) {
        xTimerStart(_timeoutTimer, 0);
    }
    
    return true;
}

/* @brief Clear the pending request
 */
void Client::PendingRequest::clear() {
    Lock guard(_mutex);
    
    // Stop timeout timer
    if (_timeoutTimer) {
        xTimerStop(_timeoutTimer, 0);
    }
    
    _reqMetadata.clear();
    _tracker = nullptr;
    _pResponse = nullptr;
    _timestampMs = 0;
    _active = false;
}

/* @brief Check if the pending request is active
 * @return true if the pending request is active
 */
const bool Client::PendingRequest::isActive() const { return _active; }

/* @brief Check if the pending request has a response pointer
 * @return true if the pending request has a response pointer
 */
const bool Client::PendingRequest::hasResponse() const { return _pResponse != nullptr; }

/* @brief Get the pending request timestamp
 * @return The timestamp of the pending request (creation time)
 */
const uint32_t Client::PendingRequest::getTimestampMs() const { return _timestampMs; }

/* @brief Get the pending request
 * @return Copy of the pending request
 */
const Modbus::Frame& Client::PendingRequest::getRequestMetadata() const { return _reqMetadata; }

/* @brief Update the pending request result tracker
 * @param result The result to set
 */
void Client::PendingRequest::setResult(Result result) { 
    Lock guard(_mutex); 
    if (_tracker) *_tracker = result; 
}

/* @brief Set the response for the pending request & update the result tracker
 * @param response The response to set
 */
void Client::PendingRequest::setResponse(const Modbus::Frame& response) { 
    Lock guard(_mutex);
    if (_pResponse) *_pResponse = response;
    if (_tracker) *_tracker = SUCCESS;
}

/* @brief Destructor for PendingRequest - cleanup timer
 */
Client::PendingRequest::~PendingRequest() {
    if (_timeoutTimer) {
        xTimerDelete(_timeoutTimer, 0);
        _timeoutTimer = nullptr;
    }
}

// ===================================================================================
// PUBLIC METHODS
// ===================================================================================

Client::Client(ModbusInterface::IInterface& interface, uint32_t timeoutMs) : 
    _interface(interface),
    _pendingRequest(),
    _requestTimeoutMs(timeoutMs),
    _isInitialized(false)
{}

Client::~Client() {
    if (_isInitialized && _handleTxResultTaskHandle != nullptr) {
        // Kill the cleanup task
        vTaskDelete(_handleTxResultTaskHandle);
        _handleTxResultTaskHandle = nullptr;
    }
    
    // Cleanup any active pending request
    _pendingRequest.clear();
    _isInitialized = false;
}


/* @brief Initialize the client & launch the pollPendingRequest task
 * @return Success if the client was initialized successfully
 * @note Launches the pollPendingRequest task in a dedicated thread
 */
Client::Result Client::begin() {
    if (_isInitialized) return Success();

    if (_interface.getRole() != Modbus::CLIENT) {
        return Error(ERR_INIT_FAILED, "interface must be CLIENT");
    }

    if (_interface.begin() != ModbusInterface::IInterface::SUCCESS) {
        return Error(ERR_INIT_FAILED, "interface init failed");
    }

    ModbusInterface::RcvCallback rcvCb = [this](const Modbus::Frame& frame) {
        handleResponse(frame);
    };

    auto setRcvCbRes = _interface.setRcvCallback(rcvCb);
    if (setRcvCbRes != ModbusInterface::IInterface::SUCCESS) {
        return Error(ERR_INIT_FAILED, "cannot set receive callback on interface");
    }

    BaseType_t cleanupTaskRes = xTaskCreatePinnedToCore(
        /*pxTaskCode*/      handleTxResultTask,
        /*pcName*/          "ModbusClient",
        /*usStackDepth*/    CLEANUP_TASK_STACK_SIZE,
        /*pvParameters*/    this,
        /*uxPriority*/      tskIDLE_PRIORITY,
        /*pvCreatedTask*/   &_handleTxResultTaskHandle,
        /*xCoreID*/         tskNO_AFFINITY
    );

    _isInitialized = true;
    return Success();
}

/* @brief Check if the client is ready to accept a new request
 * @return true if interface ready & no active pending request
 */
bool Client::isReady() {
    if (!_isInitialized) return false;
    return (_interface.isReady() && !_pendingRequest.isActive());
}

/* @brief Send a request to the interface
 * @param request The request to send
 * @param response Where to store the response (response.data will be resized to match its size)
 * @param userTracker nullptr for blocking (sync) mode, or pointer to transfer result tracker for async mode
 * @return - In async mode: Success if the transfer was started successfully (check the tracker to follow up)
 * @return - In sync mode: Success if the transfer completed successfully
 */
Client::Result Client::sendRequest(const Modbus::Frame& request, 
                         Modbus::Frame& response,
                         Result* userTracker) {
   // Reject if the frame is not a request, invalid, or another request is already in progress
    if (request.type != Modbus::REQUEST) {
        return Error(ERR_INVALID_FRAME, "non-request frame");
    }
    auto validResult = ModbusCodec::isValidFrame(request);
    if (validResult != ModbusCodec::SUCCESS) {
        return Error(ERR_INVALID_FRAME, "invalid fields");
    }
    if (!isReady()) {
        return Error(ERR_BUSY, "interface busy or active pending request");
    }

    // Choose tracker to use : if userTracker is not provided ("sync mode"), we create a local one
    Result localResult;
    Result* tracker = userTracker ? userTracker : &localResult;

    // Basic checks passed, we try to initiate the pending request
    if (!_pendingRequest.set(request, &response, tracker, _requestTimeoutMs)) {
        return Error(ERR_BUSY, "request already in progress");
    }

    _pendingRequest.setResult(NODATA); // Update result tracker immediately

    bool isBroadcast = Modbus::isBroadcastId(request.slaveId);

    // Send the frame on the interface
    auto sendRes = _interface.sendFrame(request, _handleTxResultTaskHandle);
    if (sendRes != ModbusInterface::IInterface::SUCCESS) {
        _pendingRequest.setResult(ERR_TX_FAILED);
        _pendingRequest.clear();
        return Error(ERR_TX_FAILED);
    }

    // ---------- Synchronous mode (userTracker == nullptr) ----------
    if (!userTracker) {
        // Wait until the pending request finishes (response, timeout, or TX error)
        while (_pendingRequest.isActive()) {
            WAIT_MS(1);
        }

        // The localResult variable now contains the outcome
        bool ok = (*tracker == SUCCESS);
        if (!ok) {
            // The result tracker indicates the outcome of the request
            // (TX failure or timeout -> detected by the pollPendingRequest task)
            return Error(*tracker);
        }
        return Success();
    }

    // ---------- Asynchronous mode (userTracker != nullptr) ----------
    return Success();
}

// ===================================================================================
// PRIVATE METHODS
// ===================================================================================

/* @brief Handle a response from the interface
 * @param response The response to handle
 */
void Client::handleResponse(const Modbus::Frame& response)
{
    // 1. Ignore if no request is in progress
    if (!_pendingRequest.isActive()) {
        Modbus::Debug::LOG_MSG("Received response while no request in progress, ignoring");
        return;
    }

    // 2. Check if response matches the expected FC
    if (response.type != Modbus::RESPONSE ||
        response.fc   != _pendingRequest.getRequestMetadata().fc)
    {
        Modbus::Debug::LOG_MSG("Received unexpected frame, ignoring");
        return;
    }

    // 3. Copy the response and re-inject the original metadata
    _responseBuffer.clear();
    _responseBuffer = response;
    const Modbus::Frame& req = _pendingRequest.getRequestMetadata();
    _responseBuffer.regAddress = req.regAddress;   // <- original address
    _responseBuffer.regCount   = req.regCount;     // <- actual number of registers / coils

    // 4. DO NOT unpack the coils here !
    //    Keep the 125 uint16_t array as is,
    //    it will be unpacked in the user code.

    // 5. Propagate the response to the user and clean up the state
    _pendingRequest.setResponse(_responseBuffer);
    _pendingRequest.clear();
}


/* @brief Wait for a notification from the RTU TX task (UART send outcome)
 * @param value The value to store the notification in
 * @param timeoutMs The timeout in milliseconds
 * @return true if a notification was received, false if timeout occured
 */
inline bool Client::waitTaskNotification(uint32_t& value, const uint32_t timeoutMs) {
    // Wait for a notification from the RTU TX task (UART send outcome)
    // Copies the notification value into the arg
    // Returns false if timeout occured
    return xTaskNotifyWait(0, UINT32_MAX, &value, pdMS_TO_TICKS(timeoutMs)) == pdPASS;
}

/* @brief This task is used to:
 * @brief - Raise error if the TX failed
 * @brief - Properly terminate successful broadcast requests by feeding the response callback
 * @note This task is launched in a dedicated thread by the begin() method
 * @note Timeouts are now handled by FreeRTOS timers, not by this task
 */
void Client::handleTxResultTask(void* client) {
    auto self = static_cast<Client*>(client);
    while (true) {
        // Role of this task:
        // 1. Raise error if the TX failed
        // 2. Properly terminate successful broadcast requests by feeding the response callback
        // 3. Timeouts are now handled automatically by FreeRTOS timers
        uint32_t value = 0;
        // Wait for a notification from the RTU TX task (UART send outcome)
        if (self->_pendingRequest.isActive() && self->waitTaskNotification(value)) {
            auto sendResult = static_cast<ModbusInterface::IInterface::Result>(value);
            // If it's a broadcast and the TX succeeded, it's the end of the request
            bool isBroadcast = Modbus::isBroadcastId(self->_pendingRequest.getRequestMetadata().slaveId);
            if (isBroadcast && sendResult == ModbusInterface::IInterface::SUCCESS) {
                // We feed the user callback with a dummy frame to signal the end of the request
                Modbus::Frame dummy = self->_pendingRequest.getRequestMetadata();
                dummy.type = Modbus::RESPONSE;          // transfo en "réponse"
                dummy.exceptionCode = Modbus::NULL_EXCEPTION;
                dummy.clearData(); // make sure data is empty
                self->_pendingRequest.setResponse(dummy);
                self->_pendingRequest.clear();
                continue; // rester dans la boucle de la tâche
            }
            // Otherwise, if the TX failed, it's an error - SUCCESS will be set by handleResponse()
            if (sendResult != ModbusInterface::IInterface::SUCCESS) {
                // something really went wrong at the UART/queue/encoding step
                self->_pendingRequest.setResult(ERR_TX_FAILED);
                self->_pendingRequest.clear();
            }
        }
        
        // Timeout is now handled by FreeRTOS timer, no manual checking needed
        // vTaskDelay(pdMS_TO_TICKS(50));  // Longer delay since we only handle TX notifications
    }
}


} // namespace Modbus
