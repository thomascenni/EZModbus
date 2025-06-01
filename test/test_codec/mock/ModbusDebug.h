// test/test_codec/mock/ModbusDebug.h
#ifndef MODBUS_DEBUG_H  
#define MODBUS_DEBUG_H

#include <stdio.h>

#ifdef __cplusplus
#include <iostream>
#include <string>

namespace Modbus {
namespace Debug {

    struct CallCtx {
      const char* file;
      const char* function;
      int line;
      
      CallCtx(const char* f = __builtin_FILE(), 
              const char* func = __builtin_FUNCTION(), 
              int l = __builtin_LINE()) 
          : file(f), function(func), line(l) {}
    };

} // namespace Debug
} // namespace Modbus


#ifdef EZMODBUS_DEBUG

namespace Modbus   {
namespace Debug    {

    inline const char* getBasename(const char* p) {
        const char *b = p;
        const char *s = strrchr(p, '/');  if (s) b = s + 1;
        s = strrchr(p, '\\');             if (s && s > b) b = s + 1;
        return b;
    }
    inline void LOG_MSG(const std::string& msg,
                        const CallCtx& ctx) {
        std::printf("[%s::%s:%d] %s\n",
                    ctx.file, ctx.function, ctx.line, msg.c_str());
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

} // namespace Debug
} // namespace Modbus

#endif // EZMODBUS_DEBUG

#endif  // __cplusplus

#endif  // MODBUS_DEBUG_H
