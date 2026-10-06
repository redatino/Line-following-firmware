// Copyright (c) 2026 ArduinoLFR contributors. SPDX-License-Identifier: MIT

#include "MotorChannel.h"

#include <Arduino.h>

#include "../control/DriveTypes.h"

namespace lfr {

MotorChannel::MotorChannel(const Pins& pins, bool inverted, uint8_t minPwm)
    : pins_(pins), inverted_(inverted), minPwm_(minPwm), speed_(0) {}

void MotorChannel::begin() {
  pinMode(pins_.enable, OUTPUT);
  pinMode(pins_.in1, OUTPUT);
  pinMode(pins_.in2, OUTPUT);
  coast();
}

void MotorChannel::setSpeed(int16_t speed) {
  speed = constrain(speed, -kMaxWheelSpeed, kMaxWheelSpeed);
  speed_ = speed;

  if (speed == 0) {
    coast();
    return;
  }

  const bool forward = (speed > 0) != inverted_;
  const uint16_t magnitude = static_cast<uint16_t>(speed > 0 ? speed : -speed);

  digitalWrite(pins_.in1, forward ? HIGH : LOW);
  digitalWrite(pins_.in2, forward ? LOW : HIGH);
  analogWrite(pins_.enable, applyDeadband(magnitude));
}

void MotorChannel::brake() {
  speed_ = 0;
  digitalWrite(pins_.in1, HIGH);
  digitalWrite(pins_.in2, HIGH);
  digitalWrite(pins_.enable, HIGH);
}

void MotorChannel::coast() {
  speed_ = 0;
  digitalWrite(pins_.enable, LOW);
  digitalWrite(pins_.in1, LOW);
  digitalWrite(pins_.in2, LOW);
}

uint8_t MotorChannel::applyDeadband(uint16_t magnitude) const {
  if (magnitude < minPwm_) {
    return minPwm_;
  }
  return static_cast<uint8_t>(magnitude);
}

}  // namespace lfr
