// Copyright (c) 2026 ArduinoLFR contributors. SPDX-License-Identifier: MIT

#include "LineFollowerController.h"

namespace lfr {

const char* toString(FollowerState state) {
  switch (state) {
    case FollowerState::Cruising:
      return "CRUISE";
    case FollowerState::SteeringLeft:
      return "LEFT";
    case FollowerState::SteeringRight:
      return "RIGHT";
    case FollowerState::Junction:
      return "JUNCTION";
    case FollowerState::Stopped:
      return "STOPPED";
  }
  return "?";
}

LineFollowerController::LineFollowerController(const ControllerTuning& tuning)
    : tuning_(tuning), state_(FollowerState::Cruising), stateEnteredMs_(0) {}

void LineFollowerController::reset(uint32_t nowMs) {
  state_ = FollowerState::Cruising;
  stateEnteredMs_ = nowMs;
}

WheelSpeeds LineFollowerController::update(const LineReading& reading,
                                           uint32_t nowMs) {
  if (state_ == FollowerState::Stopped) {
    return {0, 0};
  }

  const FollowerState observed = classify(reading);
  if (observed != state_) {
    transitionTo(observed, nowMs);
  }

  // Unsigned subtraction stays correct across millis() rollover.
  const uint32_t elapsedMs = nowMs - stateEnteredMs_;

  switch (state_) {
    case FollowerState::SteeringLeft:
      return {innerSpeedAfter(elapsedMs), tuning_.steerOuterSpeed};

    case FollowerState::SteeringRight:
      return {tuning_.steerOuterSpeed, innerSpeedAfter(elapsedMs)};

    case FollowerState::Junction:
      if (tuning_.stopOnMarker && elapsedMs >= tuning_.stopMarkerMs) {
        transitionTo(FollowerState::Stopped, nowMs);
        return {0, 0};
      }
      return {tuning_.cruiseSpeed, tuning_.cruiseSpeed};

    case FollowerState::Cruising:
    case FollowerState::Stopped:
      break;
  }
  return {tuning_.cruiseSpeed, tuning_.cruiseSpeed};
}

FollowerState LineFollowerController::classify(const LineReading& reading) {
  if (reading.left && reading.right) {
    return FollowerState::Junction;
  }
  if (reading.left) {
    return FollowerState::SteeringLeft;
  }
  if (reading.right) {
    return FollowerState::SteeringRight;
  }
  return FollowerState::Cruising;
}

void LineFollowerController::transitionTo(FollowerState next, uint32_t nowMs) {
  state_ = next;
  stateEnteredMs_ = nowMs;
}

int16_t LineFollowerController::innerSpeedAfter(uint32_t elapsedMs) const {
  if (tuning_.steerRampMs == 0 || elapsedMs >= tuning_.steerRampMs) {
    return tuning_.steerInnerEnd;
  }
  const int32_t span =
      static_cast<int32_t>(tuning_.steerInnerEnd) - tuning_.steerInnerStart;
  const int32_t offset =
      span * static_cast<int32_t>(elapsedMs) / tuning_.steerRampMs;
  return static_cast<int16_t>(tuning_.steerInnerStart + offset);
}

}  // namespace lfr
