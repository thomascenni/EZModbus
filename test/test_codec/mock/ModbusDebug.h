// test/test_codec/mock/ModbusDebug.h
#ifndef MODBUS_DEBUG_H  
#define MODBUS_DEBUG_H

/* ======================================================================
 * 1)  Entête commun C / C++ : uniquement de la C-lib standard
 * ====================================================================== */
#include <stdio.h>     // printf(), etc.
#include <math.h>          // ← lu AVANT nos macros => plus de conflit

/* Neutraliser d’éventuelles définitions antérieures ------------------- */
#undef  log
#undef  logln
#undef  logf

/* Macros de log                                                        */
#define log(x)        (printf("%s", (x)))
#define logln(x)      (printf("%s\n", (x)))
#define logf(...)     (printf(__VA_ARGS__))

/* ======================================================================
 * 2)  La suite n’est vue que par le compilateur C++
 * ====================================================================== */
#ifdef __cplusplus
  #include <iostream>
  #include <string>

  /* Si tu préfères cout / endl côté C++ : ----------------------------- */
  #undef  log
  #undef  logln
  #define log(x)      (std::cout << (x))
  #define logln(x)    (std::cout << (x) << std::endl)

  namespace Modbus   {
  namespace Debug    {
      inline const char* getBasename(const char* p) {
          const char *b = p;
          const char *s = strrchr(p, '/');  if (s) b = s + 1;
          s = strrchr(p, '\\');             if (s && s > b) b = s + 1;
          return b;
      }
      inline void LOG_MSG(const std::string& msg,
                          const char* file = __builtin_FILE(),
                          const char* func = __builtin_FUNCTION(),
                          int         line = __builtin_LINE()) {
          std::printf("[%s::%s:%d] %s\n",
                      getBasename(file), func, line, msg.c_str());
      }
  }} // namespaces
#endif  // __cplusplus
#endif  // MODBUS_DEBUG_H
