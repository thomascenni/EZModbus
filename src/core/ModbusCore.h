/**
 * @file ModbusCore.h
 * @brief Modbus core definitions
 */

#pragma once

#include "core/ModbusTypes.h"

namespace Modbus {

// ===================================================================================
// MODBUS CONSTANTS
// ===================================================================================

    static constexpr size_t MIN_PDU_SIZE = 1;      // FC only
    static constexpr size_t MAX_PDU_SIZE = 253;    // Excludes Slave ID & CRC (not in PDU)
    static constexpr uint16_t MAX_SLAVE_ID = 247;    // We accept up to 247 for regular requests
    static constexpr std::array<uint16_t, 1> BROADCAST_SLAVE_IDS = {0x00}; // 0 only by default (can be extended if needed)
    static constexpr size_t MAX_REGISTERS_READ = 125;   // Maximum registers per request
    static constexpr size_t MAX_REGISTERS_WRITE = 123;   // Maximum registers per request
    static constexpr size_t MAX_COILS_READ = 2000;      // Maximum coils per request
    static constexpr size_t MAX_COILS_WRITE = 1968;      // Maximum coils per request
    static constexpr uint16_t MAX_REG_ADDR = 0xFFFF; // Maximum register address
    static constexpr size_t FRAME_DATASIZE = 125; // Maximum size (registers or packed coils) of Modbus::Frame data field

// ===================================================================================
// MODBUS REGISTER TYPES
// ===================================================================================

    /* @brief The type of a register.
     * @note Representation of the register types used in the Modbus protocol.
     */
    enum RegisterType {
        NULL_RT = 0,
        COIL,
        DISCRETE_INPUT,
        HOLDING_REGISTER,
        INPUT_REGISTER
    };
    static constexpr const char* toString(const RegisterType type) {
        switch (type) {
            case DISCRETE_INPUT: return "discrete input";
            case COIL: return "coil";
            case HOLDING_REGISTER: return "holding register";
            case INPUT_REGISTER: return "input register";
            default: return "invalid register type";
        }
    }
    static bool isValid(const RegisterType type) {
        return type > NULL_RT && type <= INPUT_REGISTER;
    }

// ===================================================================================
// MODBUS FUNCTION CODES
// ===================================================================================

    /* @brief The type of a function code.
     * @note Representation of the function codes used in the Modbus protocol.
     */
    enum FunctionCode {
        NULL_FC = 0x00,
        READ_COILS = 0x01,
        READ_DISCRETE_INPUTS = 0x02,
        READ_HOLDING_REGISTERS = 0x03,
        READ_INPUT_REGISTERS = 0x04,
        WRITE_COIL = 0x05,
        WRITE_REGISTER = 0x06,
        WRITE_MULTIPLE_COILS = 0x0F,
        WRITE_MULTIPLE_REGISTERS = 0x10,
        NB_FC
    };
    static constexpr const char* toString(FunctionCode fc) {
        switch (fc) {
            case NULL_FC: return "null function code";
            case READ_COILS: return "read coils";
            case READ_DISCRETE_INPUTS: return "read discrete inputs";
            case READ_HOLDING_REGISTERS: return "read holding registers";
            case READ_INPUT_REGISTERS: return "read input registers";
            case WRITE_COIL: return "write single coil";
            case WRITE_REGISTER: return "write single register";
            case WRITE_MULTIPLE_COILS: return "write multiple coils";
            case WRITE_MULTIPLE_REGISTERS: return "write multiple registers";
            default: return "invalid function code";
        }
    }
    static bool isValid(const FunctionCode fc) {
        return fc >= NULL_FC && fc < NB_FC;
    }
    
    /* @brief Convert a function code to a register type
     * @param fc The function code to convert
     * @return The register type
     */
    static const RegisterType toRegisterType(const FunctionCode fc) {
        switch (fc) {
            case READ_COILS: 
            case WRITE_COIL:
            case WRITE_MULTIPLE_COILS:
                return RegisterType::COIL;
            case READ_DISCRETE_INPUTS:
                return RegisterType::DISCRETE_INPUT;
            case READ_HOLDING_REGISTERS:
            case WRITE_REGISTER:
            case WRITE_MULTIPLE_REGISTERS:
                return RegisterType::HOLDING_REGISTER;
            case READ_INPUT_REGISTERS:
                return RegisterType::INPUT_REGISTER;
            default:
                return RegisterType::NULL_RT; // TODO: add a NULL_TYPE
        }
    }

// ===================================================================================
// MODBUS EXCEPTION CODES
// ===================================================================================

    /* @brief The type of an exception code.
     * @note This is a representation of the exception codes used in the Modbus protocol.
     */
    enum ExceptionCode {
        NULL_EXCEPTION = 0x00,
        ILLEGAL_FUNCTION = 0x01,
        ILLEGAL_DATA_ADDRESS = 0x02,
        ILLEGAL_DATA_VALUE = 0x03,
        SLAVE_DEVICE_FAILURE = 0x04,
        ACKNOWLEDGE = 0x05,
        SLAVE_DEVICE_BUSY = 0x06,
        NEGATIVE_ACKNOWLEDGE = 0x07,
        MEMORY_PARITY_ERROR = 0x08,
        NB_EC
    };
    static constexpr const char* toString(ExceptionCode ec) {
        switch (ec) {
            case NULL_EXCEPTION: return "no exception";
            case ILLEGAL_FUNCTION: return "illegal function";
            case ILLEGAL_DATA_ADDRESS: return "illegal data address";
            case ILLEGAL_DATA_VALUE: return "illegal data value";
            case SLAVE_DEVICE_FAILURE: return "slave device failure";
            case ACKNOWLEDGE: return "acknowledge";
            case SLAVE_DEVICE_BUSY: return "slave device busy";
            case NEGATIVE_ACKNOWLEDGE: return "negative acknowledge";
            case MEMORY_PARITY_ERROR: return "memory parity error";
            default: return "invalid exception code";
        }
    }
    static bool isValid(const ExceptionCode ec) {
        return ec >= NULL_EXCEPTION && ec < NB_EC;
    }

// ===================================================================================
// MODBUS MESSAGE TYPES
// ===================================================================================

    /* @brief The type of a Modbus message.
     * @note Representation of the message types used in the Modbus protocol.
     */
    enum MsgType {
        NULL_MSG,
        REQUEST,
        RESPONSE
    };
    static constexpr const char* toString(MsgType type) {
        switch (type) {
            case NULL_MSG: return "undefined message type";
            case REQUEST: return "request";
            case RESPONSE: return "response";
            default: return "invalid message type";
        }
    }
    static bool isValid(const MsgType type) {
        return type == REQUEST || type == RESPONSE;
    }

// ===================================================================================
// MODBUS ROLES
// ===================================================================================

    /* @brief The role of a Modbus device.
     * @note Representation of the roles used in the Modbus protocol.
     * @note Legacy & modern semantics are mixed in this enum.
     */
    enum Role {
        SLAVE = 0,
        SERVER = SLAVE,  // Alias for SLAVE
        MASTER = 1,
        CLIENT = MASTER  // Alias for MASTER
    };
    static bool isValid(const Role role) {
        return role == SLAVE || role == SERVER || role == MASTER || role == CLIENT;
    }

// ===================================================================================
// MODBUS FRAME
// ===================================================================================

    /* @brief Contains the detail of a Modbus request without protocol implementation details
     * (byte count, crc & frame length are computed by codec functions)
     */
    struct Frame {
        // Data members
        Modbus::MsgType type = Modbus::NULL_MSG;
        Modbus::FunctionCode fc = Modbus::NULL_FC;
        uint8_t slaveId = 0;
        uint16_t regAddress = 0;
        uint16_t regCount = 0;
        std::array<uint16_t, FRAME_DATASIZE> data = {};
        Modbus::ExceptionCode exceptionCode = Modbus::NULL_EXCEPTION;

        // Clear functions
        void clear();
        void clearData();

        // Getter functions for frame data
        uint16_t getRegister(size_t index) const;
        std::vector<uint16_t> getRegisters() const;
        size_t getRegisters(uint16_t* dst, size_t dstSize) const;
        bool getCoil(size_t index) const;
        std::vector<bool> getCoils() const;
        size_t getCoils(bool* dst, size_t dstSize) const;

        // Setter functions for frame data
        bool setRegisters(const std::vector<uint16_t>& src);
        bool setRegisters(const std::initializer_list<uint16_t>& src);
        bool setRegisters(const uint16_t* src, size_t len);     
        bool setCoils(const std::vector<bool>& src);
        bool setCoils(const std::vector<uint16_t>& src);
        bool setCoils(const std::initializer_list<bool>& src);
        bool setCoils(const std::initializer_list<uint16_t>& src);
        bool setCoils(const bool* src, size_t len);
        bool setCoils(const uint16_t* src, size_t len);
    };

    /* @brief Clear the frame.
     * @note This function clears the frame to its default state.
     */
    inline void Frame::clear() {
        type = NULL_MSG;
        fc = NULL_FC;
        slaveId = 0;
        regAddress = 0;
        regCount = 0;
        data.fill(0);
        exceptionCode = NULL_EXCEPTION;
    }

    /* @brief Clear the data of the frame.
     * @note This function clears the data of the frame to its default state.
     */
    inline void Frame::clearData() {
        data.fill(0);
    }

    /* @brief Get a register value from frame data.
     * @param index The index of the register to get.
     * @note Returns 0 if the index is out of bounds. Check size first with regCount.
     * @return The register at the given index.
     */
    inline uint16_t Modbus::Frame::getRegister(size_t index) const {
        return (index < regCount && index < FRAME_DATASIZE) ? data[index] : 0;
    }

    /* @brief Get all registers from frame data.
     * @note Returns an empty vector if the frame has no registers.
     * @return A vector of all registers.
     */
    inline std::vector<uint16_t> Modbus::Frame::getRegisters() const {
        if (regCount == 0) return {};
        
        std::vector<uint16_t> result;
        size_t count = std::min((size_t)regCount, FRAME_DATASIZE);
        result.resize(count);
        memcpy(result.data(), data.data(), count * sizeof(uint16_t));
        return result;
    }

    /* @brief Get all registers from frame data.
     * @param dst The destination array to store the registers.
     * @param dstLen The length of the destination array.
     * @note Returns 0 if the frame has no registers or if the destination array is nullptr.
     * @note Data is converted to registers regardless of the function code.
     * @return The number of registers copied to the destination array.
     */
    inline size_t Modbus::Frame::getRegisters(uint16_t* dst, size_t dstLen) const {
        if (regCount == 0 || dst == nullptr || dstLen == 0) return 0;
        
        size_t count = std::min(regCount, (uint16_t)dstLen);
        count = std::min(count, FRAME_DATASIZE);
        
        memcpy(dst, data.data(), count * sizeof(uint16_t));
        return count;
    }

    /* @brief Get a coil value from frame data.
     * @param index The index of the coil to get.
     * @note Returns false if the index is out of bounds. Check size first with regCount.
     * @return The coil at the given index.
     */
    inline bool Modbus::Frame::getCoil(size_t index) const {
        if (index >= regCount || index >= FRAME_DATASIZE * 16) return false;
        size_t wordIdx = index / 16;
        size_t bitIdx = index % 16;
        return (data[wordIdx] >> bitIdx) & 1;
    }

    /* @brief Get all coils from frame data.
     * @note Returns an empty vector if the frame has no coils.
     * @return A vector of all coils.
     */
    inline std::vector<bool> Modbus::Frame::getCoils() const {
        if (regCount == 0) return {};
        
        std::vector<bool> result;
        size_t count = std::min((size_t)regCount,(FRAME_DATASIZE * 16));
        result.reserve(count);
        
        for (size_t i = 0; i < count; i++) {
            size_t wordIdx = i / 16;
            size_t bitIdx = i % 16;
            result.push_back((data[wordIdx] >> bitIdx) & 1);
        }
        return result;
    }

    /* @brief Get all coils from frame data.
     * @param dst The destination array to store the coils.
     * @param dstLen The length of the destination array.
     * @note Returns 0 if the frame has no coils or if the destination array is nullptr.
     * @note Data is converted to coils regardless of the function code.
     * @return The number of coils copied to the destination array.
     */
    inline size_t Modbus::Frame::getCoils(bool* dst, size_t dstLen) const {
        if (regCount == 0 || dst == nullptr || dstLen == 0) return 0;
        
        size_t count = std::min(regCount, (uint16_t)(dstLen));
        count = std::min(count, (size_t)(FRAME_DATASIZE * 16));
        
        for (size_t i = 0; i < count; i++) {
            size_t wordIdx = i / 16;
            size_t bitIdx = i % 16;
            dst[i] = (data[wordIdx] >> bitIdx) & 1;
        }
        return count;
    }


    /* @brief Pack register data from a vector into data array format.
     * @param src The source vector to pack the registers from.
     * @note Returns an empty array if the source vector is empty.
     * @return The array of registers.
     */
    inline std::array<uint16_t, FRAME_DATASIZE> packRegisters(const std::vector<uint16_t>& src) {
        std::array<uint16_t, FRAME_DATASIZE> result{};
        size_t count = std::min(src.size(), size_t(FRAME_DATASIZE));
        memcpy(result.data(), src.data(), count * sizeof(uint16_t));
        return result;
    }

    /* @brief Pack register data from an initializer list into data array format.
     * @param src The source initializer list to pack the registers from.
     * @note Returns an empty array if the source initializer list is empty.
     * @return The array of registers.
     */
    inline std::array<uint16_t, FRAME_DATASIZE> packRegisters(std::initializer_list<uint16_t> src) {
        std::array<uint16_t, FRAME_DATASIZE> result{};
        size_t i = 0;
        for (auto val : src) {
            if (i < FRAME_DATASIZE) result[i++] = val;
            else break;
        }
        return result;
    }

    /* @brief Pack register data from a buffer into data array format.
     * @param src The source buffer to pack the registers from.
     * @param len The length of the source buffer.
     * @note Returns an empty array if the source buffer is nullptr or len is 0.
     * @return The array of registers.
     */
    inline std::array<uint16_t, FRAME_DATASIZE> packRegisters(const uint16_t* src, size_t len) {
        std::array<uint16_t, FRAME_DATASIZE> result{};
        if (!src || len == 0) return result;
        size_t count = std::min(len, size_t(FRAME_DATASIZE));
        memcpy(result.data(), src, count * sizeof(uint16_t));
        return result;
    }

    /* @brief Pack coil data from a bool vector into data array format.
     * @param src The source vector to pack the coils from.
     * @note Returns an empty array if the source vector is empty.
     * @return The array of registers with packed coils.
     */
    inline std::array<uint16_t, FRAME_DATASIZE> packCoils(const std::vector<bool>& src) {
        std::array<uint16_t, FRAME_DATASIZE> result{};
        
        // Pack 16 bits at once to optimize
        for (size_t i = 0; i < src.size() && i < (FRAME_DATASIZE * 16); i += 16) {
            uint16_t packed = 0;
            for (size_t j = 0; j < 16 && (i + j) < src.size(); j++) {
                if (src[i + j]) {
                    packed |= (1u << j);
                }
            }
            result[i / 16] = packed;
        }
        return result;
    }

    /* @brief Pack coil data from a uint16_t vector into data array format (0/non-zero -> bool).
     * @param src The source vector to pack the coils from.
     * @note Returns an empty array if the source vector is empty. Any non-zero value is considered true.
     * @return The array of registers with packed coils.
     */
    inline std::array<uint16_t, FRAME_DATASIZE> packCoils(const std::vector<uint16_t>& src) {
        std::array<uint16_t, FRAME_DATASIZE> result{};
        
        // Pack 16 bits at once to optimize
        for (size_t i = 0; i < src.size() && i < (FRAME_DATASIZE * 16); i += 16) {
            uint16_t packed = 0;
            for (size_t j = 0; j < 16 && (i + j) < src.size(); j++) {
                if (src[i + j] != 0) {
                    packed |= (1u << j);
                }
            }
            result[i / 16] = packed;
        }
        return result;
    }

    /* @brief Pack coil data from a bool initializer list into data array format.
     * @param src The source initializer list to pack the coils from.
     * @note Returns an empty array if the source initializer list is empty.
     * @return The array of registers with packed coils.
     */
    inline std::array<uint16_t, FRAME_DATASIZE> packCoils(std::initializer_list<bool> src) {
        std::array<uint16_t, FRAME_DATASIZE> result{};
        
        size_t bitIndex = 0;
        for (bool bit : src) {
            if (bitIndex >= FRAME_DATASIZE * 16) break;
            
            size_t wordIdx = bitIndex / 16;
            size_t bitPos = bitIndex % 16;
            if (bit) {
                result[wordIdx] |= (1u << bitPos);
            }
            bitIndex++;
        }
        return result;
    }

    /* @brief Pack coil data from a uint16_t initializer list into data array format (0/non-zero -> bool).
     * @param src The source initializer list to pack the coils from.
     * @note Returns an empty array if the source initializer list is empty. Any non-zero value is considered true.
     * @return The array of registers with packed coils.
     */
    inline std::array<uint16_t, FRAME_DATASIZE> packCoils(std::initializer_list<uint16_t> src) {
        std::array<uint16_t, FRAME_DATASIZE> result{};
        
        size_t bitIndex = 0;
        for (uint16_t val : src) {
            if (bitIndex >= FRAME_DATASIZE * 16) break;
            
            size_t wordIdx = bitIndex / 16;
            size_t bitPos = bitIndex % 16;
            if (val != 0) {
                result[wordIdx] |= (1u << bitPos);
            }
            bitIndex++;
        }
        return result;
    }

    /* @brief Pack coil data from a bool buffer into data array format.
     * @param src The source buffer to pack the coils from.
     * @param len The length of the source buffer.
     * @note Returns an empty array if the source buffer is nullptr or len is 0.
     * @return The array of registers with packed coils.
     */
    inline std::array<uint16_t, FRAME_DATASIZE> packCoils(const bool* src, size_t len) {
        std::array<uint16_t, FRAME_DATASIZE> result{};
        if (!src || len == 0) return result;
        
        // Pack 16 bits at once to optimize
        for (size_t i = 0; i < len && i < (FRAME_DATASIZE * 16); i += 16) {
            uint16_t packed = 0;
            for (size_t j = 0; j < 16 && (i + j) < len; j++) {
                if (src[i + j]) {
                    packed |= (1u << j);
                }
            }
            result[i / 16] = packed;
        }
        return result;
    }

    /* @brief Pack coil data from a uint16_t buffer into data array format (0/non-zero -> bool).
     * @param src The source buffer to pack the coils from.
     * @param len The length of the source buffer.
     * @note Returns an empty array if the source buffer is nullptr or len is 0. Any non-zero value is considered true.
     * @return The array of registers with packed coils.
     */
    inline std::array<uint16_t, FRAME_DATASIZE> packCoils(const uint16_t* src, size_t len) {
        std::array<uint16_t, FRAME_DATASIZE> result{};
        if (!src || len == 0) return result;
        
        // Pack 16 bits at once to optimize
        for (size_t i = 0; i < len && i < (FRAME_DATASIZE * 16); i += 16) {
            uint16_t packed = 0;
            for (size_t j = 0; j < 16 && (i + j) < len; j++) {
                if (src[i + j] != 0) {
                    packed |= (1u << j);
                }
            }
            result[i / 16] = packed;
        }
        return result;
    }

    /* @brief Set register data from a vector.
     * @param src The source vector to set the registers from.
     * @note Updates both data array and regCount.
     * @return true if successful, false if the source vector is too large.
     */
    inline bool Modbus::Frame::setRegisters(const std::vector<uint16_t>& src) {
        if (src.size() > FRAME_DATASIZE) return false;
        
        data = packRegisters(src);
        regCount = std::min(src.size(), size_t(FRAME_DATASIZE));
        return true;
    }

    /* @brief Set register data from an initializer list.
     * @param src The source initializer list to set the registers from.
     * @note Updates both data array and regCount.
     * @return true if successful, false if the source list is too large.
     */
    inline bool Modbus::Frame::setRegisters(const std::initializer_list<uint16_t>& src) {
        if (src.size() > FRAME_DATASIZE) return false;
        
        data = packRegisters(src);
        regCount = std::min(src.size(), size_t(FRAME_DATASIZE));
        return true;
    }

    /* @brief Set register data from a buffer.
     * @param src The source buffer to set the registers from.
     * @param len The length of the source buffer.
     * @note Updates both data array and regCount.
     * @return true if successful, false if the source buffer is invalid or too large.
     */
    inline bool Modbus::Frame::setRegisters(const uint16_t* src, size_t len) {
        if (!src || len == 0 || len > FRAME_DATASIZE) return false;
        
        data = packRegisters(src, len);
        regCount = std::min(len, size_t(FRAME_DATASIZE));
        return true;
    }

    /* @brief Set coil data from a bool vector.
     * @param src The source vector to set the coils from.
     * @note Updates both data array and regCount.
     * @return true if successful, false if the source vector is too large.
     */
    inline bool Modbus::Frame::setCoils(const std::vector<bool>& src) {
        if (src.size() > FRAME_DATASIZE * 16) return false;
        
        data = packCoils(src);
        regCount = src.size();
        return true;
    }

    /* @brief Set coil data from a uint16_t vector (0/non-zero -> bool).
     * @param src The source vector to set the coils from.
     * @note Updates both data array and regCount. Any non-zero value is considered true.
     * @return true if successful, false if the source vector is too large.
     */
    inline bool Modbus::Frame::setCoils(const std::vector<uint16_t>& src) {
        if (src.size() > FRAME_DATASIZE * 16) return false;
        
        data = packCoils(src);
        regCount = src.size();
        return true;
    }

    /* @brief Set coil data from a bool initializer list.
     * @param src The source initializer list to set the coils from.
     * @note Updates both data array and regCount.
     * @return true if successful, false if the source list is too large.
     */
    inline bool Modbus::Frame::setCoils(const std::initializer_list<bool>& src) {
        if (src.size() > FRAME_DATASIZE * 16) return false;
        
        data = packCoils(src);
        regCount = src.size();
        return true;
    }

    /* @brief Set coil data from a uint16_t initializer list (0/non-zero -> bool).
     * @param src The source initializer list to set the coils from.
     * @note Updates both data array and regCount. Any non-zero value is considered true.
     * @return true if successful, false if the source list is too large.
     */
    inline bool Modbus::Frame::setCoils(const std::initializer_list<uint16_t>& src) {
        if (src.size() > FRAME_DATASIZE * 16) return false;
        
        data = packCoils(src);
        regCount = src.size();
        return true;
    }

    /* @brief Set coil data from a bool buffer.
     * @param src The source buffer to set the coils from.
     * @param len The length of the source buffer.
     * @note Updates both data array and regCount.
     * @return true if successful, false if the source buffer is invalid or too large.
     */
    inline bool Modbus::Frame::setCoils(const bool* src, size_t len) {
        if (!src || len == 0 || len > FRAME_DATASIZE * 16) return false;
        
        data = packCoils(src, len);
        regCount = len;
        return true;
    }

    /* @brief Set coil data from a uint16_t buffer (0/non-zero -> bool).
     * @param src The source buffer to set the coils from.
     * @param len The length of the source buffer.
     * @note Updates both data array and regCount. Any non-zero value is considered true.
     * @return true if successful, false if the source buffer is invalid or too large.
     */
    inline bool Modbus::Frame::setCoils(const uint16_t* src, size_t len) {
        if (!src || len == 0 || len > FRAME_DATASIZE * 16) return false;
        
        data = packCoils(src, len);
        regCount = len;
        return true;
    }

// ===================================================================================
// HELPER FUNCTIONS
// ===================================================================================

    /* @brief Create an ILLEGAL_FUNCTION exception response from a request frame
     * @note Copies the original request frame passed as argument
     * @param request The initial request frame
     * @return The response frame with the exception
     */
    inline Frame setIllegalFunction(const Frame& request) {
        return Frame {
            .type = RESPONSE,
            .fc = request.fc,
            .slaveId = request.slaveId,
            .regAddress = request.regAddress,
            .regCount = request.regCount,
            .data = {},
            .exceptionCode = ILLEGAL_FUNCTION
        };
    }

    /* @brief Create a SLAVE_DEVICE_BUSY exception response from a request frame
     * @note Copies the original request frame passed as argument
     * @param request The initial request frame
     * @return The response frame with the exception
     */
    inline Frame setSlaveBusy(const Frame& request) {
        return Frame {
            .type = RESPONSE,
            .fc = request.fc,
            .slaveId = request.slaveId,
            .regAddress = request.regAddress,
            .regCount = request.regCount,
            .data = {},
            .exceptionCode = SLAVE_DEVICE_BUSY
        };
    }

    /* @brief Create a SLAVE_DEVICE_FAILURE exception response from a request frame
     * @note Copies the original request frame passed as argument
     * @param request The initial request frame
     * @return The response frame with the exception
     */
    inline Frame setSlaveDeviceFailure(const Frame& request) {
        return Frame {
            .type = RESPONSE,
            .fc = request.fc,
            .slaveId = request.slaveId,
            .regAddress = request.regAddress,
            .regCount = request.regCount,
            .data = {},
            .exceptionCode = SLAVE_DEVICE_FAILURE
        };
    }

    /* @brief Check if a slave ID is a broadcast address
     * @brief ATTENTION: this function checks ONLY the Slave ID, not the message type (req/resp) or the FC (read/write).
     * @param slaveId The slave ID to check
     * @return true if it's a broadcast address, false otherwise
     */
    inline bool isBroadcastId(const uint8_t slaveId) {
        for (auto id : BROADCAST_SLAVE_IDS) {
            if (slaveId == id) return true;
        }
        return false;
    }

} // namespace Modbus