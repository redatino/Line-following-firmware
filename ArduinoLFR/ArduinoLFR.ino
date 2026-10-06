// Copyright (c) 2026 ArduinoLFR contributors. SPDX-License-Identifier: MIT
//
// ArduinoLFR: two-sensor line following robot (Arduino Uno + L298N).
//
// Layering:
//   hal/      thin wrappers over pins (IR sensors, motor channels, LED)
//   drive/    differential drive with slew limiting
//   control/  hardware-independent state machine (unit-tested on host)
//   Config.h  every pin and tuning constant
//
// The loop is non-blocking and runs the controller at a fixed period.

#include "src/Config.h"
#include "src/control/LineFollowerController.h"
#include "src/drive/DifferentialDrive.h"
#include "src/hal/IrSensor.h"
#include "src/hal/MotorChannel.h"
#include "src/hal/StatusLed.h"
#include "src/util/Log.h"

namespace cfg = lfr::config;

namespace {

lfr::IrSensor leftSensor(cfg::kPinSensorLeft, cfg::kSensorLineLevel,
                         cfg::kSensorDebounceSamples);
lfr::IrSensor rightSensor(cfg::kPinSensorRight, cfg::kSensorLineLevel,
                          cfg::kSensorDebounceSamples);

lfr::MotorChannel leftMotor({cfg::kPinMotorLeftEnable, cfg::kPinMotorLeftIn1,
                             cfg::kPinMotorLeftIn2},
                            cfg::kMotorLeftInverted, cfg::kMotorMinPwm);
lfr::MotorChannel rightMotor({cfg::kPinMotorRightEnable, cfg::kPinMotorRightIn1,
                              cfg::kPinMotorRightIn2},
                             cfg::kMotorRightInverted, cfg::kMotorMinPwm);

lfr::DifferentialDrive drive(leftMotor, rightMotor, cfg::kMotorSlewPerTick);

lfr::LineFollowerController controller({
    cfg::kCruiseSpeed,
    cfg::kSteerOuterSpeed,
    cfg::kSteerInnerStart,
    cfg::kSteerInnerEnd,
    cfg::kSteerRampMs,
    cfg::kStopOnMarker,
    cfg::kStopMarkerMs,
});

lfr::StatusLed statusLed(cfg::kPinStatusLed);

enum class RunMode : uint8_t { Arming, Running, Halted };

RunMode runMode = RunMode::Arming;
uint32_t bootMs = 0;
uint32_t lastTickMs = 0;

#if LFR_DEBUG
uint32_t lastTelemetryMs = 0;

void logTelemetry(uint32_t nowMs, const lfr::LineReading& reading,
                  const lfr::WheelSpeeds& speeds) {
  if (nowMs - lastTelemetryMs < cfg::kTelemetryPeriodMs) {
    return;
  }
  lastTelemetryMs = nowMs;
  LFR_LOGF("t=%lu L=%d R=%d state=%s pwm=%d,%d",
           static_cast<unsigned long>(nowMs), reading.left, reading.right,
           lfr::toString(controller.state()), speeds.left, speeds.right);
}
#endif

void controlTick(uint32_t nowMs) {
  leftSensor.sample();
  rightSensor.sample();
  const lfr::LineReading reading{leftSensor.onLine(), rightSensor.onLine()};

  const lfr::WheelSpeeds speeds = controller.update(reading, nowMs);

  if (controller.state() == lfr::FollowerState::Stopped) {
    drive.brake();
    statusLed.blink(200);
    runMode = RunMode::Halted;
    LFR_LOGF("stop marker detected, halting");
    return;
  }

  drive.drive(speeds);

#if LFR_DEBUG
  logTelemetry(nowMs, reading, speeds);
#endif
}

}  // namespace

void setup() {
  LFR_LOG_BEGIN();

  drive.begin();
  leftSensor.begin();
  rightSensor.begin();
  statusLed.begin();
  statusLed.blink(500);  // slow blink: armed, waiting to start

  bootMs = millis();
  LFR_LOGF("ArduinoLFR ready, starting in %u ms", cfg::kStartDelayMs);
}

void loop() {
  const uint32_t nowMs = millis();
  statusLed.update(nowMs);

  switch (runMode) {
    case RunMode::Arming:
      if (nowMs - bootMs >= cfg::kStartDelayMs) {
        controller.reset(nowMs);
        lastTickMs = nowMs;
        statusLed.on();
        runMode = RunMode::Running;
        LFR_LOGF("running");
      }
      break;

    case RunMode::Running:
      if (nowMs - lastTickMs >= cfg::kControlPeriodMs) {
        lastTickMs += cfg::kControlPeriodMs;
        controlTick(nowMs);
      }
      break;

    case RunMode::Halted:
      // Press the board's reset button to run again.
      break;
  }
}
