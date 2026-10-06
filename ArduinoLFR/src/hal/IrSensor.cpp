// Copyright (c) 2026 ArduinoLFR contributors. SPDX-License-Identifier: MIT

#include "IrSensor.h"

#include <Arduino.h>

namespace lfr {

IrSensor::IrSensor(uint8_t pin, bool lineLevel, uint8_t debounceSamples)
    : pin_(pin),
      lineLevel_(lineLevel),
      debounceSamples_(debounceSamples == 0 ? 1 : debounceSamples),
      onLine_(false),
      candidate_(false),
      candidateCount_(0) {}

void IrSensor::begin() {
  pinMode(pin_, INPUT);
  onLine_ = (digitalRead(pin_) == HIGH) == lineLevel_;
  candidate_ = onLine_;
  candidateCount_ = debounceSamples_;
}

void IrSensor::sample() {
  const bool raw = (digitalRead(pin_) == HIGH) == lineLevel_;

  if (raw != candidate_) {
    candidate_ = raw;
    candidateCount_ = 1;
  } else if (candidateCount_ < debounceSamples_) {
    ++candidateCount_;
  }

  if (candidateCount_ >= debounceSamples_) {
    onLine_ = candidate_;
  }
}

}  // namespace lfr
