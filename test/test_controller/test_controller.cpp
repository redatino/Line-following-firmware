// Copyright (c) 2026 ArduinoLFR contributors. SPDX-License-Identifier: MIT
//
// Host-side unit tests for the line follower state machine.
// Run with: pio test -e native

#include <unity.h>

#include "src/control/LineFollowerController.h"

using lfr::FollowerState;
using lfr::LineFollowerController;
using lfr::LineReading;
using lfr::WheelSpeeds;

namespace {

const lfr::ControllerTuning kTuning = {
    /*cruiseSpeed=*/150,
    /*steerOuterSpeed=*/200,
    /*steerInnerStart=*/100,
    /*steerInnerEnd=*/-100,
    /*steerRampMs=*/200,
    /*stopOnMarker=*/true,
    /*stopMarkerMs=*/100,
};

const LineReading kNone = {false, false};
const LineReading kLeft = {true, false};
const LineReading kRight = {false, true};
const LineReading kBoth = {true, true};

void assertSpeeds(int16_t left, int16_t right, const WheelSpeeds& actual) {
  TEST_ASSERT_EQUAL_INT16(left, actual.left);
  TEST_ASSERT_EQUAL_INT16(right, actual.right);
}

void assertState(FollowerState expected, FollowerState actual) {
  TEST_ASSERT_EQUAL_STRING(lfr::toString(expected), lfr::toString(actual));
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_cruises_when_line_is_centred() {
  LineFollowerController c(kTuning);
  c.reset(0);
  assertSpeeds(150, 150, c.update(kNone, 10));
  assertState(FollowerState::Cruising, c.state());
}

void test_steers_towards_left_sensor() {
  LineFollowerController c(kTuning);
  c.reset(0);
  assertSpeeds(100, 200, c.update(kLeft, 10));
  assertState(FollowerState::SteeringLeft, c.state());
}

void test_steers_towards_right_sensor() {
  LineFollowerController c(kTuning);
  c.reset(0);
  assertSpeeds(200, 100, c.update(kRight, 10));
  assertState(FollowerState::SteeringRight, c.state());
}

void test_inner_wheel_ramps_linearly_then_saturates() {
  LineFollowerController c(kTuning);
  c.reset(0);
  c.update(kLeft, 1000);
  assertSpeeds(0, 200, c.update(kLeft, 1100));     // halfway
  assertSpeeds(-100, 200, c.update(kLeft, 1200));  // fully ramped
  assertSpeeds(-100, 200, c.update(kLeft, 5000));  // stays saturated
}

void test_ramp_restarts_after_switching_sides() {
  LineFollowerController c(kTuning);
  c.reset(0);
  c.update(kLeft, 0);
  c.update(kLeft, 500);
  assertSpeeds(200, 100, c.update(kRight, 510));
}

void test_short_junction_is_driven_through() {
  LineFollowerController c(kTuning);
  c.reset(0);
  assertSpeeds(150, 150, c.update(kBoth, 0));
  assertSpeeds(150, 150, c.update(kBoth, 99));
  assertSpeeds(150, 150, c.update(kNone, 105));
  assertState(FollowerState::Cruising, c.state());
}

void test_long_junction_latches_stop() {
  LineFollowerController c(kTuning);
  c.reset(0);
  c.update(kBoth, 0);
  assertSpeeds(0, 0, c.update(kBoth, 100));
  assertState(FollowerState::Stopped, c.state());
  assertSpeeds(0, 0, c.update(kNone, 200));  // stays stopped
}

void test_reset_clears_stop() {
  LineFollowerController c(kTuning);
  c.reset(0);
  c.update(kBoth, 0);
  c.update(kBoth, 100);
  c.reset(300);
  assertSpeeds(150, 150, c.update(kNone, 305));
}

void test_stop_marker_can_be_disabled() {
  lfr::ControllerTuning tuning = kTuning;
  tuning.stopOnMarker = false;
  LineFollowerController c(tuning);
  c.reset(0);
  c.update(kBoth, 0);
  assertSpeeds(150, 150, c.update(kBoth, 10000));
  assertState(FollowerState::Junction, c.state());
}

void test_timing_survives_millis_rollover() {
  LineFollowerController c(kTuning);
  const uint32_t nearWrap = 0xFFFFFFFFUL - 50;
  c.reset(nearWrap);
  c.update(kLeft, nearWrap);
  // 100 ms later, after the counter has wrapped: halfway through the ramp.
  assertSpeeds(0, 200, c.update(kLeft, nearWrap + 100));
}

void test_zero_ramp_goes_straight_to_end_speed() {
  lfr::ControllerTuning tuning = kTuning;
  tuning.steerRampMs = 0;
  LineFollowerController c(tuning);
  c.reset(0);
  assertSpeeds(-100, 200, c.update(kLeft, 0));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_cruises_when_line_is_centred);
  RUN_TEST(test_steers_towards_left_sensor);
  RUN_TEST(test_steers_towards_right_sensor);
  RUN_TEST(test_inner_wheel_ramps_linearly_then_saturates);
  RUN_TEST(test_ramp_restarts_after_switching_sides);
  RUN_TEST(test_short_junction_is_driven_through);
  RUN_TEST(test_long_junction_latches_stop);
  RUN_TEST(test_reset_clears_stop);
  RUN_TEST(test_stop_marker_can_be_disabled);
  RUN_TEST(test_timing_survives_millis_rollover);
  RUN_TEST(test_zero_ramp_goes_straight_to_end_speed);
  return UNITY_END();
}
