// Copyright (c) 2026 ArduinoLFR contributors. SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

namespace lfr {

// Non-blocking status LED: solid on, off, or blinking at a given period.
class StatusLed {
 public:
  explicit StatusLed(uint8_t pin);

  void begin();
  void on();
  void off();
  void blink(uint16_t periodMs);

  // Drives the blink pattern. Call every loop iteration.
  void update(uint32_t nowMs);

 private:
  void write(bool lit);

  uint8_t pin_;
  uint16_t periodMs_;  // 0 = steady
  bool steadyLit_;
  bool lit_;
  uint32_t lastToggleMs_;
};

}  // namespace lfr
