// Copyright (c) 2026 ArduinoLFR contributors. SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

namespace lfr {

// Digital reflective IR sensor (TCRT5000, FC-51 and similar) with a simple
// consecutive-sample debounce.
class IrSensor {
 public:
  // lineLevel: the pin level (true = HIGH) the module outputs over the line.
  // debounceSamples: identical samples required to accept a change (>= 1).
  IrSensor(uint8_t pin, bool lineLevel, uint8_t debounceSamples);

  void begin();

  // Takes one sample. Call once per control tick.
  void sample();

  bool onLine() const { return onLine_; }

 private:
  uint8_t pin_;
  bool lineLevel_;
  uint8_t debounceSamples_;
  bool onLine_;
  bool candidate_;
  uint8_t candidateCount_;
};

}  // namespace lfr
