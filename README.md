# ArduinoLFR

Firmware for a two-sensor line following robot: Arduino Uno, two digital IR
reflectance sensors, two DC gear motors on an L298N dual H-bridge.

- Smooth steering from on/off sensors: the inner wheel ramps from a gentle
  arc to a pivot the longer a sensor sits on the line.
- Stop-marker detection: a thick cross bar stops the robot; thin cross-overs
  are driven straight through.
- Non-blocking 200 Hz control loop, sensor debounce, motor deadband
  compensation and slew-rate limiting.
- Hardware-independent controller with host-side unit tests and CI.
- One source tree that builds in both PlatformIO and the Arduino IDE.

## Hardware

| Part | Notes |
|---|---|
| Arduino Uno (ATmega328P) | Nano works with the same pin map |
| L298N dual H-bridge module | ENA/ENB jumpers **removed** so the Uno drives PWM |
| 2x digital IR sensor (TCRT5000 / FC-51) | Digital `DO`/`OUT` pin, not analogue |
| 2x DC gear motor + wheels, caster | Typical 3-6 V "TT" motors |
| Battery pack, 7-12 V | 2S Li-ion or 6xAA. L298N drops ~2 V |

### Wiring

All pins are defined in [`ArduinoLFR/src/Config.h`](ArduinoLFR/src/Config.h).

| Signal | Arduino pin | Connects to |
|---|---|---|
| Left IR sensor `OUT` | D2 | Left sensor |
| Right IR sensor `OUT` | D4 | Right sensor |
| Left motor enable (PWM) | D5 | L298N `ENA` |
| Left motor dir A | D7 | L298N `IN1` |
| Left motor dir B | D8 | L298N `IN2` |
| Right motor enable (PWM) | D6 | L298N `ENB` |
| Right motor dir A | D9 | L298N `IN3` |
| Right motor dir B | D10 | L298N `IN4` |
| Status LED | D13 | On-board LED |

```
 Battery +  ──────────────┬──────────── L298N +12V
                          │
                          └──────────── Arduino VIN   (or use L298N 5V out)
 Battery −  ───── L298N GND ─────────── Arduino GND   (common ground: required)

 L298N OUT1/OUT2 ── left motor          L298N OUT3/OUT4 ── right motor
 IR sensors: VCC ── Arduino 5V, GND ── Arduino GND
```

Sensor placement: mount both sensors at the front, 3-8 mm above the floor,
spaced slightly **wider than the line** so both read "off line" when the robot
is centred. For 19 mm electrical tape, roughly 25-30 mm between sensor centres
works well.

> Supply above 12 V: remove the L298N's 5 V regulator jumper and power the Uno
> separately.

## Build and flash

### PlatformIO (recommended)

```sh
pio run -e uno -t upload          # release build
pio run -e uno_debug -t upload    # with serial telemetry at 115200 baud
pio device monitor
pio test -e native                # unit tests on your PC, no hardware needed
```

### Arduino IDE

Open `ArduinoLFR/ArduinoLFR.ino`, select **Arduino Uno**, upload. The IDE
compiles everything under `ArduinoLFR/src/` automatically. To enable
telemetry, set `#define LFR_DEBUG 1` at the top of `Config.h`.

## Operation

1. Power on and set the robot on the line. The LED blinks slowly for 2 s.
2. LED goes solid: the robot is running.
3. LED blinks fast: stop marker reached. Press reset to run again.

### How it steers

| Left | Right | State | Action |
|---|---|---|---|
| off | off | `CRUISE` | both wheels at cruise speed |
| on | off | `LEFT` | arc left; inner wheel ramps down to reverse |
| off | on | `RIGHT` | arc right; inner wheel ramps down to reverse |
| on | on | `JUNCTION` | drive through; if it persists for `kStopMarkerMs`, stop |

## Tuning

Every knob is in `Config.h`. Change one at a time.

| Symptom | Adjust |
|---|---|
| Steers **away** from the line | Flip `kSensorLineLevel` |
| A wheel spins backwards | Set `kMotorLeftInverted` / `kMotorRightInverted` |
| Wheels hum but do not turn at low speed | Raise `kMotorMinPwm` |
| Zig-zags on straights | Raise `kSteerInnerStart`, lengthen `kSteerRampMs` |
| Runs wide on sharp corners | Lower `kSteerInnerEnd` (more negative), shorten `kSteerRampMs`, lower `kCruiseSpeed` |
| Stops at cross-overs | Raise `kStopMarkerMs` or set `kStopOnMarker = false` |
| Jerky starts | Lower `kMotorSlewPerTick` |

Check the sensors first: most modules have an on-board LED and a trim pot.
Adjust the pot until the LED changes state cleanly between line and floor.

## Project layout

```
ArduinoLFR/
  ArduinoLFR.ino                 entry point: setup/loop, run modes, telemetry
  src/
    Config.h                     pins and tuning (edit this)
    control/                     pure logic, no Arduino dependencies
      DriveTypes.h
      LineFollowerController.*   steering state machine
    drive/
      DifferentialDrive.*        two-wheel drive with slew limiting
    hal/
      IrSensor.*                 debounced digital IR sensor
      MotorChannel.*             one L298N channel (PWM, direction, brake)
      StatusLed.*                non-blocking LED patterns
    util/
      Log.h                      compile-time switchable serial logging
test/test_controller/            Unity tests for the controller (host)
platformio.ini                   uno, uno_debug and native environments
.github/workflows/ci.yml         PlatformIO build/test + arduino-cli compile
```

The control layer takes a sensor reading and a timestamp and returns wheel
speeds. It never touches pins, which is what makes it testable on a PC.

## License

MIT. See [LICENSE](LICENSE).
