// Copyright (c) 2026 ArduinoLFR contributors. SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

#include "DriveTypes.h"

namespace lfr {

enum class FollowerState : uint8_t {
  Cruising,       // line between the sensors: drive straight
  SteeringLeft,   // left sensor on the line: arc left, harder over time
  SteeringRight,  // right sensor on the line: arc right, harder over time
  Junction,       // both sensors on the line: cross-over or stop marker
  Stopped,        // stop marker confirmed; latched until reset()
};

const char* toString(FollowerState state);

struct ControllerTuning {
  int16_t cruiseSpeed;
  int16_t steerOuterSpeed;
  int16_t steerInnerStart;
  int16_t steerInnerEnd;
  uint16_t steerRampMs;
  bool stopOnMarker;
  uint16_t stopMarkerMs;
};

// Two-sensor line follower state machine.
//
// The sensors straddle the line. When one of them touches the line the robot
// arcs towards that side; the inner wheel slows linearly from steerInnerStart
// to steerInnerEnd over steerRampMs, so small drifts get gentle corrections and
// sharp corners escalate to a pivot. This avoids the zig-zag of a pure
// bang-bang controller without needing analogue sensors.
//
// The controller is pure logic: it takes a reading and a timestamp and returns
// wheel speeds. It never touches hardware, which keeps it unit-testable.
class LineFollowerController {
 public:
  explicit LineFollowerController(const ControllerTuning& tuning);

  // Clears any latched stop and restarts timing from nowMs.
  void reset(uint32_t nowMs);

  // Advances the state machine. nowMs may wrap; all timing is wrap-safe.
  WheelSpeeds update(const LineReading& reading, uint32_t nowMs);

  FollowerState state() const { return state_; }

 private:
  static FollowerState classify(const LineReading& reading);
  void transitionTo(FollowerState next, uint32_t nowMs);
  int16_t innerSpeedAfter(uint32_t elapsedMs) const;

  ControllerTuning tuning_;
  FollowerState state_;
  uint32_t stateEnteredMs_;
};

}  // namespace lfr
