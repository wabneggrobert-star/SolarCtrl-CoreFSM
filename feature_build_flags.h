#pragma once
#include <Arduino.h>

#include "feature_build_flags.h"
#ifndef SOLARCTRL_SERIAL_DEBUG
#define SOLARCTRL_SERIAL_DEBUG 0
#endif

#ifndef SOLARCTRL_MAX_DIAGNOSTICS
#define SOLARCTRL_MAX_DIAGNOSTICS 0
#endif

#if SOLARCTRL_SERIAL_DEBUG
  #define DBG_PRINT(...)   DBG_PRINT(__VA_ARGS__)
  #define DBG_PRINTLN(...) DBG_PRINTLN(__VA_ARGS__)
  #define DBG_PRINTF(...)  DBG_PRINTF(__VA_ARGS__)
  #define DBG_FLUSH()      DBG_FLUSH()
#else
  #define DBG_PRINT(...)   do {} while (0)
  #define DBG_PRINTLN(...) do {} while (0)
  #define DBG_PRINTF(...)  do {} while (0)
  #define DBG_FLUSH()      do {} while (0)
#endif
