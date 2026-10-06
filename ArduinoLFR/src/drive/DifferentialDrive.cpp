// Copyright (c) 2026 ArduinoLFR contributors. SPDX-License-Identifier: MIT

#include "DifferentialDrive.h"

namespace lfr {

DifferentialDrive::DifferentialDrive(MotorChannel& left, MotorChannel& right,
                                     uint8_t maxStepPerTick)
    : left_(left), right_(right), maxStep_(maxStepPerTick) {}

void DifferentialDrive::begin() {
  left_.begin();
  right_.begin();
}

void DifferentialDrive::drive(const WheelSpeeds& target) {
  left_.setSpeed(approach(left_.speed(), target.left, maxStep_));
  right_.setSpeed(approach(right_.speed(), target.right, maxStep_));
}

void DifferentialDrive::brake() {
  left_.brake();
  right_.brake();
}

void DifferentialDrive::coast() {
  left_.coast();
  right_.coast();
}

int16_t DifferentialDrive::approach(int16_t current, int16_t target,
                                    int16_t maxStep) {
  if (target > current + maxStep) {
    return current + maxStep;
  }
  if (target < current - maxStep) {
    return current - maxStep;
  }
  return target;
}

}  // namespace lfr
