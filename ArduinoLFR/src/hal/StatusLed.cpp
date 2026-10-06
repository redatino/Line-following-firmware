// Copyright (c) 2026 ArduinoLFR contributors. SPDX-License-Identifier: MIT

#include "StatusLed.h"

#include <Arduino.h>

namespace lfr {

StatusLed::StatusLed(uint8_t pin)
    : pin_(pin), periodMs_(0), steadyLit_(false), lit_(false), lastToggleMs_(0) {}

void StatusLed::begin() {
  pinMode(pin_, OUTPUT);
  write(false);
}

void StatusLed::on() {
  periodMs_ = 0;
  steadyLit_ = true;
  write(true);
}

void StatusLed::off() {
  periodMs_ = 0;
  steadyLit_ = false;
  write(false);
}

void StatusLed::blink(uint16_t periodMs) {
  periodMs_ = periodMs;
}

void StatusLed::update(uint32_t nowMs) {
  if (periodMs_ == 0) {
    return;
  }
  if (nowMs - lastToggleMs_ >= periodMs_ / 2) {
    lastToggleMs_ = nowMs;
    write(!lit_);
  }
}

void StatusLed::write(bool lit) {
  lit_ = lit;
  digitalWrite(pin_, lit ? HIGH : LOW);
}

}  // namespace lfr
