/**
 * @file ModbusServer.cpp
 * @brief Modbus server implementation
 */

#include "ModbusServer.h"

namespace Modbus {

// ===================================================================================
// PUBLIC METHODS
// ===================================================================================

Server::Server(ModbusInterface::IInterface& interface, uint8_t slaveId, bool rejectUndefined) 
    : _interface(interface), _serverId(slaveId), _rejectUndefined(rejectUndefined) {
    // Reserve the stores for the default max number of registers
    _discreteInputStore.reserve(RSV_REGISTERS);
    _coilStore.reserve(RSV_REGISTERS);
    _inputRegisterStore.reserve(RSV_REGISTERS);
    _holdingRegisterStore.reserve(RSV_REGISTERS);
    _cbStore.reserve(RSV_REGISTERS);
}

/* @brief Initialize the server
 * @return Result::SUCCESS if the server was initialized successfully
 * @note The server is fed by interface callbacks, no polling required
 */
Server::Result Server::begin() {
    if (_interface.getRole() != Modbus::SERVER) {
        return Error(Server::ERR_INIT_FAILED, "interface must be SERVER");
    }

    ModbusInterface::RcvCallback rcvCb = [this](const Modbus::Frame& frame) {
        Modbus::Debug::LOG_MSG("Received request from interface");
        handleRequest(frame);
    };

    auto setRcvCbRes = _interface.setRcvCallback(rcvCb);
    if (setRcvCbRes != ModbusInterface::IInterface::SUCCESS) {
        return Error(Server::ERR_INIT_FAILED, "cannot set receive callback on interface");
    }

    return Success();
}

/* @brief Reserve memory for a given number of registers
 * @param type The type of register to reserve memory for
 * @param count The number of registers to reserve memory for
 * @return Result::SUCCESS if operation completed
 * @note The server must be initialized before it can handle requests
 */
Server::Result Server::setRegisterCount(const Modbus::RegisterType type, const uint16_t count) {
    if (count > MAX_REGISTERS) {
        char errBuf[64];
        snprintf(errBuf, sizeof(errBuf), "Cannot reserve more than %d registers", MAX_REGISTERS);
        return Error(Server::ERR_REG_OVERFLOW, errBuf);
    }

    Lock guard(_registerStoreMutex);
    if (!guard.isLocked()) return Error(Server::ERR_REG_BUSY);

    auto regStore = findRegisterStore(type);
    if (!regStore) return Error(Server::ERR_REG_INVALID_TYPE);

    regStore->reserve(count);
    return Success();
}

/* @brief Add a register to the server
 * @param reg The register to add
 * @return Result::SUCCESS if the register was added successfully
 * @note Adding a register at an existing address will overwrite it
 */
Server::Result Server::addRegister(const Server::Register& reg) {
    Lock guard(_registerStoreMutex, 0);  // Try-lock avec timeout = 0
    if (!guard.isLocked()) return Error(Server::ERR_REG_BUSY);
    
    Server::Result res = isValidEntry(reg);
    if (res != Server::SUCCESS) {
        // Print error message: "<ERROR_CODE> : <reg.type> <reg.address>"
        char errBuf[64];
        snprintf(errBuf, sizeof(errBuf), "%s %d", Modbus::toString(reg.type), reg.address);
        return Error(res, errBuf);
    }

    // Check overflow only if register does not already exist
    if (!registerExists(reg) && wouldOverflow(reg.type, 1)) {
        return Error(Server::ERR_REG_OVERFLOW);
    }

    addRegisterInternal(reg);
    return Success();
}

/* @brief Add a set of registers to the server
 * @param registers The vector containing the registers to add
 * @return Result::SUCCESS if the registers were added successfully
 * @note Atomic operation: all registers will be added or none!
 */
Server::Result Server::addRegisters(const std::vector<Server::Register>& registers) {
    Lock guard(_registerStoreMutex, 0);  // Try-lock avec timeout = 0
    if (!guard.isLocked()) return Error(Server::ERR_REG_BUSY);

    // Count registers by type (invalid type + legit types)
    std::array<size_t, 5> newCount = {0};
    
    // Check validity of each register
    for (const auto& reg : registers) {
        Server::Result res = isValidEntry(reg);
        if (res != Server::SUCCESS) {
            // Print error message: "<ERROR_CODE> : <reg.type> <reg.address>"
            char errBuf[64];
            snprintf(errBuf, sizeof(errBuf), "%s %d", Modbus::toString(reg.type), reg.address);
            return Error(res, errBuf);
        }

        if (!registerExists(reg)) {
            newCount[reg.type]++;
        }
    }

    // Check overflow for each register type
    for (int type = 1; type <= 4; type++) {
        if (wouldOverflow(static_cast<Modbus::RegisterType>(type), newCount[type])) {
            return Error(Server::ERR_REG_OVERFLOW);
        }
    }

    // Add each register to the store
    for (const auto& reg : registers) {
        addRegisterInternal(reg);
    }
    
    return Success();
}

/* @brief Clear all registers from the server
 * @return Result::SUCCESS if the registers were cleared successfully
 * @note Atomic operation: all registers will be cleared or none!
 */
Server::Result Server::clearAllRegisters() {
    Lock guard(_registerStoreMutex, 0);  // Try-lock avec timeout = 0
    if (!guard.isLocked()) return Error(Server::ERR_REG_BUSY);

    // Remove all registers & their callbacks
    _holdingRegisterStore.clear();
    _inputRegisterStore.clear();
    _coilStore.clear();
    _discreteInputStore.clear();
    _cbStore.clear();
    
    return Success();
}

/* @brief Get a register from the server
 * @param type The type of register to get
 * @param address The address of the register to get
 * @return The copy of the register if found, otherwise an empty register
 */
Server::Register Server::getRegister(Modbus::RegisterType type, uint16_t address) {
    auto reg = findRegisterEntry(type, address);
    if (!reg) return Server::Register();
    return reg->toRegister(this);
}

// TODO: remove this method
void Server::poll() {
}

/* @brief Check if the server is busy
 * @return true if the server is busy (adding a register or processing a request)
 */
bool Server::isBusy() {
    Lock l1(_handleRequestMutex, 0);
    Lock l2(_registerStoreMutex,     0);
    return !l1.isLocked() || !l2.isLocked();   // busy si au moins un est pris ailleurs
}

// ===================================================================================
// PRIVATE DATA STRUCTURES METHODS
// ===================================================================================

/* @brief Convert a register entry to a register
 * @param srv The server object
 * @return The copy of the register
 */
Server::Register Server::RegisterEntry::toRegister(Server* srv) {
    Server::Register regOut;
    regOut.type = type;
    regOut.address = address;
    regOut.name = name;
    regOut.value = value;
    if (cbIndex != UINT32_MAX && srv) {
        regOut.readCb = srv->findCBEntry(cbIndex)->readCb;
        regOut.writeCb = srv->findCBEntry(cbIndex)->writeCb;
    }
    return regOut;
}

// ===================================================================================
// PRIVATE METHODS
// ===================================================================================

/* @brief Direct read callback
 * @param r The register to read
 * @return The value of the register
 * @note Offers the same signature as read callbacks but in C function pointer format
 *       (for static allocation)
 */
static uint16_t _directReadCb(const Server::Register& r) {
    if (r.value) return static_cast<uint16_t>(*r.value & 0xFFFF); // Take the lower 16 bits
    return 0;
}

/* @brief Direct write callback
 * @param value The value to write
 * @param r The register to write
 * @return true if the write was successful, false otherwise
 * @note Offers the same signature as write callbacks but in C function pointer format
 *       (for static allocation)
 */
static bool _directWriteCb(uint16_t value, const Server::Register& r) {
    if (r.value) { 
        *r.value = (*r.value & 0xFFFF0000) | value;  // Only update the lower 16 bits
        return true; 
    }
    return false;
}

/* @brief Find the appropriate register store
 * @param type The type of register to find
 * @return The pointer to the register store
 */
Server::RegisterStore* Server::findRegisterStore(Modbus::RegisterType type) {
    switch (type) {
        case Modbus::HOLDING_REGISTER: return &_holdingRegisterStore;
        case Modbus::INPUT_REGISTER:   return &_inputRegisterStore;
        case Modbus::COIL:            return &_coilStore;
        case Modbus::DISCRETE_INPUT:   return &_discreteInputStore;
        default:              return nullptr;
    }
}

/* @brief Find the appropriate register store
 * @param fc The function code to find
 * @return The pointer to the register store
 */
Server::RegisterStore* Server::findRegisterStore(Modbus::FunctionCode fc) {
    return findRegisterStore(Modbus::toRegisterType(fc));
}

/* @brief Find the appropriate register entry
 * @param type The type of register to find
 * @param address The address of the register to find
 * @return The pointer to the register entry
 */
Server::RegisterEntry* Server::findRegisterEntry(const Modbus::RegisterType type, const uint16_t address) {
    auto regStore = findRegisterStore(type);
    if (!regStore) return nullptr;
    return findRegisterEntry(regStore, address);
}

/* @brief Find the appropriate register entry
 * @param regStore The register store to find the entry in
 * @param address The address of the register to find
 * @return The pointer to the register entry
 */
Server::RegisterEntry* Server::findRegisterEntry(Server::RegisterStore* regStore, const uint16_t address) {
    auto it = std::lower_bound(
        regStore->begin(), regStore->end(), address, 
        [](auto const& r, uint16_t addr) {
            return r.address < addr;
        });
    return it != regStore->end() && it->address == address ? &*it : nullptr;
}

/* @brief Find the appropriate callback entry
 * @param cbIndex The index of the callback to find
 * @return The pointer to the callback entry
 */
Server::CBEntry* Server::findCBEntry(const uint32_t cbIndex) {
    if (cbIndex == UINT32_MAX) return nullptr;
    if (cbIndex >= _cbStore.size()) return nullptr;
    return &_cbStore[cbIndex];
}

/* @brief Find the appropriate read callback
 * @param reg The register to find the callback for
 * @return The pointer to the read callback
 */
Server::ReadCallback Server::findReadCallback(const Server::RegisterEntry& reg) {
    // If the register has a value pointer, read it using our static function pointer
    if (reg.value) return _directReadCb;

    // Otherwise, return the stored callback or nullptr if not found
    auto cbEntry = findCBEntry(reg.cbIndex);
    if (!cbEntry) return nullptr;
    return cbEntry->readCb;
}

/* @brief Find the appropriate write callback
 * @param reg The register to find the callback for
 * @return The pointer to the write callback
 */
Server::WriteCallback Server::findWriteCallback(const Server::RegisterEntry& reg) {
    // If the register has a value pointer, write it using our static function pointer
    if (reg.value) return _directWriteCb;

    // Otherwise, return the stored callback or nullptr if not found
    auto cbEntry = findCBEntry(reg.cbIndex);
    if (!cbEntry) return nullptr;
    return cbEntry->writeCb;
}

/* @brief Check if a register exists
 * @param reg The register to check
 * @return true if the register exists, false otherwise
 */
bool Server::registerExists(const Server::Register& reg) {
    auto getReg = findRegisterEntry(reg.type, reg.address);
    return getReg != nullptr;
}

/* @brief Check if adding a register would overflow the register store
 * @param type The type of register to check
 * @param additionalCount The number of additional registers to add
 * @return true if the addition would overflow, false otherwise
 */
bool Server::wouldOverflow(const Modbus::RegisterType type, const size_t additionalCount) {
    auto regStore = findRegisterStore(type);
    if (!regStore) return false;
    return regStore->size() + additionalCount > MAX_REGISTERS;
}

/* @brief Add a register to the register store
 * @param reg The register to add
 */
void Server::addRegisterInternal(const Server::Register& reg) {
    // Register callbacks if provided
    uint32_t cbIdx = UINT32_MAX;
    if (reg.readCb || reg.writeCb) {
        cbIdx = _cbStore.size();
        _cbStore.emplace_back(reg.readCb, reg.writeCb);
    }

    // Find the appropriate register store
    Server::RegisterStore* regStore = findRegisterStore(reg.type);
    if (!regStore) return;

    // Do a sorted insertion by address
    auto it = std::lower_bound(
        regStore->begin(), regStore->end(), reg.address, 
        [](auto const& r, uint16_t addr) {
            return r.address < addr;
        });

    // Then add the register & the callback index
    auto insertedIt = regStore->emplace(it, reg.type, reg.address, 
                                        reg.name, reg.value, cbIdx);
    insertedIt->cbIndex = cbIdx;
}

/* @brief Process a Modbus request
 * @param request The request to process
 * @return The result of the request
 */
Server::Result Server::handleRequest(const Modbus::Frame& request) {
    bool dropResponse = false;
    Modbus::Frame response;

    { // _handleRequestMutex scope
    Lock guard(_handleRequestMutex);
    if (!guard.isLocked()) {
        // A request is already being processed, ignore this one
        return Error(Server::ERR_RCV_BUSY);
    }

    // Ignore the Slave ID if the request is sent to a broadcast slave ID
    // or if the server is configured to catch all requests (broadcast mode or TCP server)
    bool catchAllMode = Modbus::isBroadcastId(_serverId) || _interface.checkCatchAllSlaveIds();
    bool broadcastRequest = Modbus::isBroadcastId(request.slaveId);
    dropResponse = broadcastRequest;  // We don't respond to broadcast requests
    bool dropRequest = !catchAllMode 
                        && (request.slaveId != _serverId)
                        && !broadcastRequest; // We drop requests not addressed to us

    // We ignore requests not addressed to us & unsolicited responses
    if (dropRequest) return Error(Server::ERR_RCV_WRONG_SLAVE_ID);
    if (request.type != Modbus::REQUEST) return Error(Server::ERR_RCV_INVALID_TYPE);

    // Reject read requests in broadcast mode according to Modbus spec
    bool isWrite = (request.fc == Modbus::WRITE_REGISTER || 
                    request.fc == Modbus::WRITE_MULTIPLE_REGISTERS ||
                    request.fc == Modbus::WRITE_COIL ||
                    request.fc == Modbus::WRITE_MULTIPLE_COILS);
    if (broadcastRequest && !isWrite) {
        return Error(Server::ERR_RCV_ILLEGAL_FUNCTION);
    }

    // Prepare the response
    response.type = Modbus::RESPONSE;
    response.fc = request.fc;
    response.slaveId = request.slaveId;
    response.regAddress = request.regAddress;
    response.regCount = request.regCount;
    response.clearData();
    response.exceptionCode = Modbus::NULL_EXCEPTION;

    // Check that the function code is valid
    if (!ModbusCodec::isValidFunctionCode(static_cast<uint8_t>(request.fc))) {
        response.exceptionCode = Modbus::ILLEGAL_FUNCTION;
        if (!dropResponse) _interface.sendFrame(response);
        return Error(Server::ERR_RCV_ILLEGAL_FUNCTION);
    }

    // Find the appropriate register map
    auto* regStore = findRegisterStore(request.fc);
    if (!regStore) {
        response.exceptionCode = Modbus::ILLEGAL_FUNCTION;
        if (!dropResponse) _interface.sendFrame(response);
        return Error(Server::ERR_RCV_ILLEGAL_FUNCTION);
    }

    // Process the request based on whether it's a read or write
    if (isWrite) {
        handleWrite(request, response, regStore);
    } else {
        handleRead(request, response, regStore);
    }

    } // _handleRequestMutex scope

    // Send the response (unless broadcast)
    if (!dropResponse) {
        Server::Result res = sendResponse(response);
        if (res != Server::SUCCESS) return res;
    }

    return Success();
}

/* @brief Handle a read request
 * @param request The request to handle
 * @param response The response to send
 * @param regStore The register store to use
 * @return The result of the request
 */
Server::Result Server::handleRead(const Modbus::Frame& request, Modbus::Frame& response,
                        Server::RegisterStore* regStore) {
    response.clearData();

    if (!regStore) {
        response.exceptionCode = Modbus::ILLEGAL_FUNCTION;
        return Error(Server::ERR_RCV_ILLEGAL_FUNCTION);
    }

    // Check that the final address doesn't exceed the valid range
    uint32_t endAddress = static_cast<uint32_t>(request.regAddress) + request.regCount;
    if (endAddress > Modbus::MAX_REG_ADDR) {
        response.exceptionCode = Modbus::ILLEGAL_DATA_ADDRESS;
        return Error(Server::ERR_RCV_ILLEGAL_DATA_ADDRESS, "Final address exceeds valid range");
    }

    // If _rejectUndefined is true, first check that all registers exist & have a valid read callback
    if (_rejectUndefined) {
        for (uint16_t i = 0; i < request.regCount; ++i) {
            uint16_t addr = request.regAddress + i;
            auto entry = findRegisterEntry(regStore, addr);
            if (!entry || !findReadCallback(*entry)) {
                response.exceptionCode = Modbus::ILLEGAL_DATA_ADDRESS;
                return Error(Server::ERR_RCV_ILLEGAL_DATA_ADDRESS, "Register not found or unreadable");
            }
        }
    }

    // Special case for coils and discrete inputs: pack bits
    if (request.fc == Modbus::READ_COILS || request.fc == Modbus::READ_DISCRETE_INPUTS) {
        // 1) gather coil values as bools
        std::vector<bool> bits;
        bits.reserve(request.regCount);
        for (uint16_t i = 0; i < request.regCount; ++i) {
            auto entry = findRegisterEntry(regStore, request.regAddress + i);
            bool val = false;
            if (entry) {
                auto cb = findReadCallback(*entry);
                if (cb) {
                    val = cb(entry->toRegister(this)) != 0;
                }
            }
            bits.push_back(val);
        }
        
        // 2) pack into response.data
        response.data = Modbus::packCoils(bits);
        return Success();
    }

    // Bulk read holding or input registers
    for (uint16_t i = 0; i < request.regCount; ++i) {
        uint16_t addr = request.regAddress + i;
        auto entry = findRegisterEntry(regStore, addr);
        uint16_t value = 0;
        if (entry) {
            auto cb = findReadCallback(*entry);
            if (!cb) {
                response.exceptionCode = Modbus::ILLEGAL_DATA_ADDRESS;
                return Error(Server::ERR_RCV_ILLEGAL_DATA_ADDRESS, "Invalid read callback");
            }
            value = cb(entry->toRegister(this));
        }
        response.data[i] = value;
    }

    return Success();
}


/* @brief Handle a write request
 * @param request The request to handle
 * @param response The response to send
 * @param regStore The register store to use
 * @return The result of the request
 */
Server::Result Server::handleWrite(const Modbus::Frame& request, Modbus::Frame& response,
                        Server::RegisterStore* regStore) {
    if (!regStore) {
        response.exceptionCode = Modbus::ILLEGAL_FUNCTION;
        return Error(Server::ERR_RCV_ILLEGAL_FUNCTION, "Invalid register store");
    }

    // Check that the data size is valid to be sure we will write the correct number of registers 
    // UPDATE: we now assume regCount will define the no. of registers to write, no the other way round
    // if (request.data.empty() 
    //     || (request.data.size() == 1 && request.regCount > 1)
    //     || (request.data.size() != request.regCount && request.regCount > 1)) {
    //     response.exceptionCode = Modbus::ILLEGAL_DATA_VALUE;
    //     return Error(Server::ERR_RCV_ILLEGAL_DATA_VALUE, "Invalid data size");
    // }
    if (request.regCount == 0) {
        response.exceptionCode = Modbus::ILLEGAL_DATA_VALUE;
        return Error(Server::ERR_RCV_ILLEGAL_DATA_VALUE, "Invalid data size");
    }

    // Store the number of registers to write
    uint16_t writeCount = request.regCount ? request.regCount : 1;

    // Check that the final address doesn't exceed the valid range
    uint32_t endAddress = static_cast<uint32_t>(request.regAddress) + writeCount;
    if (endAddress > Modbus::MAX_REG_ADDR) {
        response.exceptionCode = Modbus::ILLEGAL_DATA_ADDRESS;
        return Error(Server::ERR_RCV_ILLEGAL_DATA_ADDRESS, "Final address exceeds valid range");
    }

    // If _rejectUndefined is true, first check that all registers exist
    // have a valid write callback
    if (_rejectUndefined) {
        for (uint16_t i = 0; i < writeCount; i++) {
            uint16_t address = request.regAddress + i;
            auto reg = findRegisterEntry(regStore, address);
            if (!reg) {
                response.exceptionCode = Modbus::ILLEGAL_DATA_ADDRESS;
                return Error(Server::ERR_RCV_ILLEGAL_DATA_ADDRESS, "Register not found in store");
            }
            if (!findWriteCallback(*reg)) {
                response.exceptionCode = Modbus::ILLEGAL_DATA_ADDRESS;
                return Error(Server::ERR_RCV_ILLEGAL_DATA_ADDRESS, "Register has no write callback");
            }
        }
    }

    // Write the registers
    for (uint16_t i = 0; i < writeCount; ++i) {
        uint16_t address = request.regAddress + i;
        auto reg = findRegisterEntry(regStore, address);
        if (!reg) continue;                // silently ignore if _rejectUndefined == false

        Server::WriteCallback cb = findWriteCallback(*reg);
        if (!cb) {
            response.exceptionCode = Modbus::ILLEGAL_DATA_ADDRESS;
            return Error(ERR_RCV_ILLEGAL_DATA_ADDRESS, "Invalid write callback");
        }

        uint16_t v = request.data[i];
        if (request.fc == WRITE_COIL || request.fc == WRITE_MULTIPLE_COILS) v = v ? 1 : 0;
        if (!cb(v, reg->toRegister(this))) {
            response.exceptionCode = Modbus::SLAVE_DEVICE_FAILURE;
            return Error(ERR_RCV_SLAVE_DEVICE_FAILURE, "Write callback returned false");
        }
    }


    // Echo data only if everything went well
    if (request.fc == Modbus::WRITE_REGISTER || request.fc == Modbus::WRITE_COIL) {
        // response.data = request.data; UPDATE
        memcpy(response.data.data(), request.data.data(), writeCount * sizeof(uint16_t));
    }
    // For multiple writes, return only the address and count
    else {
        response.regAddress = request.regAddress;
        response.regCount = writeCount;
    }
    
    return Success();
}

Server::Result Server::sendResponse(const Modbus::Frame& response) {
    auto sendResult = _interface.sendFrame(response);
    if (sendResult != ModbusInterface::IInterface::SUCCESS) {
        return Error(Server::ERR_RSP_TX_FAILED);
    }
    return Success();
}

Server::Result Server::isValidEntry(const Server::Register& reg) {
    // Check that the register type is valid
    if (!Modbus::isValid(reg.type)) {
        return Error(Server::ERR_REG_INVALID_TYPE);
    }

    // If the register has a value pointer, it's valid
    if (reg.value) return Success();
    
    // Check that the register has a defined read callback
    if (reg.readCb == nullptr) {
        return Error(Server::ERR_REG_MISSING_READCB);
    }

    // Check that the register has a defined write callback if it's not read-only
    if (!isReadOnly(reg.type) && reg.writeCb == nullptr) {
        return Error(Server::ERR_REG_MISSING_WRITECB);
    }

    // Check that the register does not have a write callback if it's read-only
    if (isReadOnly(reg.type) && reg.writeCb != nullptr) {
        return Error(Server::ERR_REG_READONLY_WRITECB);
    }

    return Success();
}

/* @brief Check if a function code is a write request
 * @param fc The function code to check
 * @return true if the function code is a write request, false otherwise
 */
bool Server::isWrite(const Modbus::FunctionCode fc) {
    return fc == Modbus::WRITE_REGISTER ||
            fc == Modbus::WRITE_COIL ||
            fc == Modbus::WRITE_MULTIPLE_REGISTERS ||
            fc == Modbus::WRITE_MULTIPLE_COILS;
}

/* @brief Check if a register type is read-only
 * @param type The register type to check
 * @return true if the register type is read-only, false otherwise
 */
bool Server::isReadOnly(const Modbus::RegisterType type) {
    return type == Modbus::DISCRETE_INPUT ||
            type == Modbus::INPUT_REGISTER;
}

} // namespace Modbus