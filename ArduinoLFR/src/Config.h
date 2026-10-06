// Copyright (c) 2026 ArduinoLFR contributors. SPDX-License-Identifier: MIT
//
// Hardware map and tuning constants. This is the only file that should need
// editing to adapt the firmware to a different chassis or wiring.

#pragma once

#include <stdint.h>

#ifndef LFR_DEBUG
#define LFR_DEBUG 0  // 1 = stream telemetry over Serial (see util/Log.h)
#endif

namespace lfr {
namespace config {

// ---------------------------------------------------------------------------
// Pin assignments (Arduino Uno). ENA/ENB must be PWM-capable pins.
// ---------------------------------------------------------------------------
constexpr uint8_t kPinSensorLeft = 2;
constexpr uint8_t kPinSensorRight = 4;

constexpr uint8_t kPinMotorLeftEnable = 5;  // L298N ENA (PWM)
constexpr uint8_t kPinMotorLeftIn1 = 7;     // L298N IN1
constexpr uint8_t kPinMotorLeftIn2 = 8;     // L298N IN2

constexpr uint8_t kPinMotorRightEnable = 6;  // L298N ENB (PWM)
constexpr uint8_t kPinMotorRightIn1 = 9;     // L298N IN3
constexpr uint8_t kPinMotorRightIn2 = 10;    // L298N IN4

constexpr uint8_t kPinStatusLed = 13;  // on-board LED

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------
// Logic level the IR module outputs while it is over the line. Most
// TCRT5000 / FC-51 style boards pull the output LOW when they see a
// reflective (white) surface, so a dark line reads HIGH. Flip this if the
// robot steers away from the line instead of towards it.
constexpr bool kSensorLineLevel = true;  // true = HIGH means "on line"

// Consecutive identical samples required before a sensor changes state.
// 1 disables debouncing.
constexpr uint8_t kSensorDebounceSamples = 2;

// ---------------------------------------------------------------------------
// Motors
// ---------------------------------------------------------------------------
// Set to true if a wheel spins backwards; cheaper than re-wiring.
constexpr bool kMotorLeftInverted = false;
constexpr bool kMotorRightInverted = false;

// Lowest PWM duty that reliably turns the wheel. Non-zero commands below this
// are lifted to it so the robot does not stall in gentle corrections.
constexpr uint8_t kMotorMinPwm = 60;

// Maximum PWM change per control tick. Limits wheel slip and current spikes.
// 255 disables slew limiting.
constexpr uint8_t kMotorSlewPerTick = 40;

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------
constexpr uint16_t kControlPeriodMs = 5;   // 200 Hz
constexpr uint16_t kStartDelayMs = 2000;   // time to set the robot down

// Speeds are signed PWM duty, -255..255.
constexpr int16_t kCruiseSpeed = 170;       // line centred between sensors
constexpr int16_t kSteerOuterSpeed = 190;   // outer wheel during a correction
constexpr int16_t kSteerInnerStart = 90;    // inner wheel when correction begins
constexpr int16_t kSteerInnerEnd = -120;    // inner wheel once fully ramped
constexpr uint16_t kSteerRampMs = 250;      // time to ramp start -> end

// Both sensors on the line for at least this long is treated as a stop
// marker. Shorter contacts (cross-overs, junctions) are driven straight over.
constexpr bool kStopOnMarker = true;
constexpr uint16_t kStopMarkerMs = 150;

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------
constexpr uint32_t kSerialBaud = 115200;
constexpr uint16_t kTelemetryPeriodMs = 100;

}  // namespace config
}  // namespace lfr
