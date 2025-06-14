/**
 * @file ModbusDebug.h
 * @brief Modbus debug utilities
 */

#pragma once

#include "core/ModbusCore.h"

namespace Modbus {
namespace Debug {

/* @brief Context structure to capture call location information
 */
struct CallCtx {
    const char* file;
    const char* function;
    int line;
    
    CallCtx(const char* f = __builtin_FILE(), 
            const char* func = __builtin_FUNCTION(), 
            int l = __builtin_LINE()) 
        : file(f), function(func), line(l) {}
};

/* @brief Maximum size for a formatted debug message (including null terminator) */
constexpr std::size_t MAX_DEBUG_MSG_SIZE = 512;

} // namespace Debug
} // namespace Modbus

#ifdef EZMODBUS_DEBUG

#include "utils/ModbusLogger.h"

namespace Modbus {
namespace Debug {

/* @brief Utility function to extract the filename from a full path
 * @param path The full path to extract the filename from
 * @return The filename
 */
static const char* getBasename(const char* path) {
    const char* basename = path;
    
    // Search for the last occurrence of '/' (Unix/Linux/macOS)
    const char* lastSlash = strrchr(path, '/');
    if (lastSlash) basename = lastSlash + 1;
    
    // Search for the last occurrence of '\' (Windows)
    const char* lastBackslash = strrchr(path, '\\');
    if (lastBackslash && lastBackslash > basename) basename = lastBackslash + 1;
    
    return basename;
}

/* @brief Format and log a debug message with printf-style formatting
 * @param ctx Call context (file, function, line)
 * @param format Printf-style format string
 * @param args Arguments for the format string
 */
template<typename... Args>
inline void LOG_MSGF_impl(CallCtx ctx, const char* format, Args&&... args) {
    // Format directly into a fixed-size stack buffer and truncate if necessary
    char buffer[MAX_DEBUG_MSG_SIZE];

    // snprintf guarantees null-termination if buffer size > 0
    int written = snprintf(buffer, sizeof(buffer), format, std::forward<Args>(args)...);
    if (written < 0) return; // snprintf error

    // Determine suffix to indicate truncation if needed
    const char* suffix = (written >= static_cast<int>(sizeof(buffer))) ? " ..." : "";

    // Single logf call with context information
    Modbus::Logger::logf("[%s::%s:%d] %s%s",
                         getBasename(ctx.file), ctx.function, ctx.line,
                         buffer, suffix);
}

// Macro to automatically capture call context
#define LOG_MSGF(format, ...) LOG_MSGF_impl(Modbus::Debug::CallCtx(), format, ##__VA_ARGS__)

inline void LOG_MSG(const std::string& message = "", CallCtx ctx = CallCtx()) {
    Modbus::Logger::logf("[%s::%s:%d] %s\n", getBasename(ctx.file), ctx.function, ctx.line, message.c_str());
}

inline void LOG_HEXDUMP(const ByteBuffer& bytes, const char* desc, CallCtx ctx = CallCtx()) {
    if (bytes.empty()) {
        Modbus::Logger::logf("[%s::%s:%d] %s<empty>\n", getBasename(ctx.file), ctx.function, ctx.line, desc);
        return;
    }

    // 1) Compute the needed size : 
    //    - for each byte : 2 chars hex + 1 space
    //    - for the prefix and a final \n
    size_t needed = strlen(desc) + (bytes.size() * 3) + 2;
    std::vector<char> buffer(needed);
    char* ptr = buffer.data();
    size_t remaining = needed;

    // 2) Copy the prefix
    int n = snprintf(ptr, remaining, "%s", desc);
    ptr += n;
    remaining -= n;

    // 3) Format each byte
    for (uint8_t b : bytes) {
        n = snprintf(ptr, remaining, "%02X ", b);
        ptr += n;
        remaining -= n;
    }

    // 4) Terminate with a newline
    if (remaining > 0) {
        *ptr++ = '\n';
        *ptr = '\0';
    } else {
        buffer.back() = '\0';
    }

    // 5) Only one logf call for everything
    Modbus::Logger::logf("[%s::%s:%d] %s", getBasename(ctx.file), ctx.function, ctx.line, buffer.data());
}

inline void LOG_FRAME(const Modbus::Frame& frame, const char* desc = nullptr, CallCtx ctx = CallCtx()) {
    // Log header with file/function/line information
    Modbus::Logger::logf("[%s::%s:%d] %s:\n", getBasename(ctx.file), ctx.function, ctx.line, desc);
    
    // Body
    Modbus::Logger::logf("> Type           : %s\n", frame.type == Modbus::REQUEST ? "REQUEST" : "RESPONSE");
    Modbus::Logger::logf("> Function code  : 0x%02X\n", frame.fc);
    Modbus::Logger::logf("> Slave ID       : %d\n", frame.slaveId);
    Modbus::Logger::logf("> Register Addr  : %d\n", frame.regAddress);
    Modbus::Logger::logf("> Register Count : %d\n", frame.regCount);
    
    // Frame data (only if not empty)
    if (frame.regCount > 0) {
        std::string dataStr = "> Data           : ";
        for (int i=0; i<frame.regCount; i++) {
            char buffer[10];
            snprintf(buffer, sizeof(buffer), "0x%04X ", frame.data[i]);
            dataStr += buffer;
        }
        Modbus::Logger::logf("%s\n", dataStr.c_str());
    }
    
    // Exception code (if present)
    if (frame.exceptionCode != Modbus::NULL_EXCEPTION) {
        Modbus::Logger::logf("> Exception     : 0x%02X\n", frame.exceptionCode);
    }
    
}

} // namespace Debug
} // namespace Modbus


#else // EZMODBUS_DEBUG


namespace Modbus {
namespace Debug {

    // If EZMODBUS_DEBUG is not defined, use no-op templates to totally
    // disable calls to LOG_xxx functions (will not evaluate args)

    template<typename... Args>
    inline void LOG_MSG(Args&&...) {}
        
    template<typename... Args>
    inline void LOG_HEXDUMP(Args&&...) {}

    template<typename... Args>
    inline void LOG_FRAME(Args&&...) {}

    template<typename... Args>
    inline void LOG_MSGF(Args&&...) {}

} // namespace Debug
} // namespace Modbus


#endif // EZMODBUS_DEBUG