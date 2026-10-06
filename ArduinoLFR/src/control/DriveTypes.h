// Copyright (c) 2026 ArduinoLFR contributors. SPDX-License-Identifier: MIT
//
// Plain data types shared between the sensing, control and drive layers.
// Deliberately free of Arduino dependencies so the control layer can be
// compiled and unit-tested on a host machine.

#pragma once

#include <stdint.h>

namespace lfr {

// Snapshot of the two line sensors. true = sensor is over the line.
struct LineReading {
  bool left;
  bool right;
};

// Signed wheel commands in PWM duty units, -255 (full reverse) .. 255.
struct WheelSpeeds {
  int16_t left;
  int16_t right;
};

constexpr int16_t kMaxWheelSpeed = 255;

}  // namespace lfr
