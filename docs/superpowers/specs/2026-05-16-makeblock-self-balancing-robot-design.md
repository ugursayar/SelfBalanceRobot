# MakeBlock Self-Balancing Robot Arduino Design

Date: 2026-05-16

## Goal

Build an Arduino project for a true two-wheel MakeBlock self-balancing robot using MegaPi, the MegaPi Shield for RJ25, two MegaPi Encoder/DC drivers, the Me 3-Axis Accelerometer and Gyro Sensor, a MakeBlock Bluetooth controller/adapter, and the Me Ultrasonic Sensor.

The first build is a cautious bring-up version: it should boot safely, calibrate when armed, balance in place, accept conservative drive commands, and clamp forward motion when an obstacle is detected.

## Hardware Mapping

The default configuration matches the current robot wiring:

- Controller board: MegaPi, programmed as Arduino Mega 2560.
- RJ25 shield: MegaPi Shield for RJ25.
- Gyro sensor: RJ25 shield `PORT_6`.
- Ultrasonic sensor: RJ25 shield `PORT_7`.
- Bluetooth: standard MakeBlock Bluetooth adapter, serial interface configurable.
- Right motor: MegaPi Encoder/DC driver in MegaPi port 1.
- Left motor: MegaPi Encoder/DC driver in MegaPi port 2.

Right and left are defined from the viewpoint of looking in the same direction as the robot.

## Architecture

The sketch is organized into small modules with one clear responsibility each:

- `SelfBalanceRobot.ino`: Arduino entry point. Owns global setup and calls fixed-rate update functions.
- `config.h`: all hardware ports, motor inversion flags, PID gains, thresholds, loop timing, and feature flags.
- `RobotState`: owns the state machine: `DISARMED`, `CALIBRATING`, `BALANCING`, `DRIVE`, and `FAULT`.
- `Sensors`: initializes and reads the gyro, ultrasonic sensor, and future encoder hooks.
- `BalanceController`: computes angle PID output from the calibrated upright angle.
- `DriveMixer`: combines balance output with forward and turn commands, applies motor inversion and output limits, then commands the motors.
- `BluetoothControl`: parses controller input into arm, stop, forward, turn, and optional tuning commands.

The first version keeps encoder feedback as a hook rather than a required control loop. This keeps the bring-up path smaller while leaving a clean place for a later speed or position loop.

## Control Strategy

The first controller uses a fixed-rate angle PID loop, targeted around 100 Hz.

During calibration, the robot samples the gyro while the chassis is held upright and still. The averaged angle becomes the upright setpoint. In `BALANCING`, the PID output drives both wheels in the same direction to keep the chassis upright. In `DRIVE`, Bluetooth forward and backward commands shift the target angle slightly, while turn commands add opposite left/right offsets.

The code is structured so a later outer encoder-speed loop can adjust the angle target without rewriting the balance loop.

## Bluetooth Behavior

Bluetooth supports both safety and drive behavior:

- Startup state is `DISARMED`; motors are off.
- Arm command starts calibration, then enters balance mode if calibration succeeds.
- Stop command immediately disables motor output and returns to `DISARMED`.
- Drive mode allows forward, backward, and turn commands only while balancing is active.
- Optional tuning commands may adjust PID values at runtime if enabled in config.

If the exact controller packet format differs from the MakeBlock examples, `BluetoothControl` is isolated so parsing can be changed without touching balance logic.

## Ultrasonic Obstacle Avoidance

The ultrasonic sensor is active while the robot is balancing. When distance is below the configured obstacle threshold:

- Forward drive command is clamped to zero.
- Backward command remains allowed so the robot can retreat.
- Turning remains allowed.
- The balance loop continues running.
- Stop and fault handling always take priority.

The ultrasonic sample rate is lower than the balance loop rate to avoid blocking or jitter in the PID loop.

## Safety Behavior

Motors are disabled whenever the robot is not in an active balancing state.

Safety rules:

- `DISARMED`: motors off.
- `CALIBRATING`: motors off.
- `BALANCING` and `DRIVE`: motors enabled only if gyro data is fresh and tilt is within limits.
- `FAULT`: motors off until the user explicitly stops or resets.
- Excessive tilt enters `FAULT`.
- Stale sensor readings enter `FAULT`.
- Motor outputs are limited by configurable min, max, and deadband values.

## Configuration Defaults

`config.h` provides conservative defaults:

- `GYRO_PORT = PORT_6`
- `ULTRASONIC_PORT = PORT_7`
- `RIGHT_MOTOR_PORT = PORT1`
- `LEFT_MOTOR_PORT = PORT2`
- Motor inversion flags for each side.
- Balance loop period.
- PID gains.
- Max motor output.
- Fall angle limit.
- Obstacle threshold in centimeters.
- Bluetooth serial selection.
- Debug serial output flag.

The goal is for tuning and wiring corrections to happen in `config.h`, not inside control logic.

## Bring-Up And Testing

Bring-up happens in stages:

1. Sensor/debug mode: confirm gyro angle changes, ultrasonic distance reads correctly, Bluetooth commands are received, and motor directions are correct.
2. Held balance test: enable balancing while holding the robot and confirm wheel response direction.
3. PID tuning: adjust gains until correction is stable and not violent.
4. Short free-standing balance tests: test in brief armed sessions with stop available.
5. Drive enablement: enable forward/backward and turning commands after balance is stable.
6. Obstacle clamp: verify forward command is blocked near an obstacle while backward and stop remain available.

Because a balancing robot can move suddenly during tuning, the sketch favors explicit arming, conservative default motor limits, and fast stop/fault paths.

## References

- MakeBlock Ultimate 2.0 Arduino setup: https://support.makeblock.com/hc/en-us/articles/1500003548142-Program-Ultimate-2-0-in-Arduino
- MakeBlock self-balancing robot case: https://support.makeblock.com/hc/en-us/articles/7314775112471-Case-11-Simulate-a-self-balancing-robot
- Me Ultrasonic Sensor: https://support.makeblock.com/hc/en-us/articles/16579980385047-About-Me-Ultrasonic-Sensor
- MegaPi: https://support.makeblock.com/hc/en-us/articles/12963818051991-About-MegaPi
- MegaPi Shield for RJ25: https://support.makeblock.com/hc/en-us/articles/12545001703703-About-MegaPi-Shield-for-RJ25
- MegaPi Encoder/DC Driver V1: https://support.makeblock.com/hc/en-us/articles/12544997605271-About-MegaPi-Encoder-DC-Driver-V1
- Me 3-Axis Accelerometer and Gyro Sensor: https://support.makeblock.com/hc/en-us/articles/12880946059159-About-Me-3-Axis-Accelerometer-and-Gyro-Sensor-for-Ultimate-2-0
