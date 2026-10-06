// Copyright (c) 2026 ArduinoLFR contributors. SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

namespace lfr {

// One H-bridge channel of an L298N (ENx + two INx pins).
//
//   INa  INb  ENx   result
//   H    L    PWM   forward
//   L    H    PWM   reverse
//   H    H    H     brake (motor terminals shorted)
//   x    x    L     coast (bridge disabled)
class MotorChannel {
 public:
  struct Pins {
    uint8_t enable;  // must be PWM-capable
    uint8_t in1;
    uint8_t in2;
  };

  MotorChannel(const Pins& pins, bool inverted, uint8_t minPwm);

  // Configures pins and leaves the motor coasting.
  void begin();

  // Signed speed, -255..255. Values outside the range are clamped. Non-zero
  // magnitudes below minPwm are raised to minPwm to overcome static friction.
  void setSpeed(int16_t speed);

  void brake();
  void coast();

  int16_t speed() const { return speed_; }

 private:
  uint8_t applyDeadband(uint16_t magnitude) const;

  Pins pins_;
  bool inverted_;
  uint8_t minPwm_;
  int16_t speed_;
};

}  // namespace lfr
