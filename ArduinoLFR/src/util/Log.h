// Copyright (c) 2026 ArduinoLFR contributors. SPDX-License-Identifier: MIT
//
// Compile-time switchable serial logging. With LFR_DEBUG == 0 every macro
// expands to nothing, so release builds carry no Serial code or strings.

#pragma once

#include "../Config.h"

#if LFR_DEBUG
#include <Arduino.h>
#include <stdio.h>

#define LFR_LOG_BEGIN() Serial.begin(::lfr::config::kSerialBaud)

// printf-style; output is truncated at 96 characters.
#define LFR_LOGF(...)                            \
  do {                                           \
    char lfrLogBuf_[96];                         \
    snprintf(lfrLogBuf_, sizeof(lfrLogBuf_), __VA_ARGS__); \
    Serial.println(lfrLogBuf_);                  \
  } while (0)
#else
#define LFR_LOG_BEGIN() \
  do {                  \
  } while (0)
#define LFR_LOGF(...) \
  do {                \
  } while (0)
#endif
