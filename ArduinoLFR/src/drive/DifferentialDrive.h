// Copyright (c) 2026 ArduinoLFR contributors. SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

#include "../control/DriveTypes.h"
#include "../hal/MotorChannel.h"

namespace lfr {

// Two-wheel differential drive with per-tick slew-rate limiting.
class DifferentialDrive {
 public:
  DifferentialDrive(MotorChannel& left, MotorChannel& right,
                    uint8_t maxStepPerTick);

  void begin();

  // Moves each wheel towards its target by at most maxStepPerTick.
  void drive(const WheelSpeeds& target);

  // Immediate stop, bypassing slew limiting.
  void brake();
  void coast();

 private:
  static int16_t approach(int16_t current, int16_t target, int16_t maxStep);

  MotorChannel& left_;
  MotorChannel& right_;
  int16_t maxStep_;
};

}  // namespace lfr
