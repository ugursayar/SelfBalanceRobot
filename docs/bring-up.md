# Bring-Up Guide

Use short tests and keep the robot held securely until motor direction and balance correction direction are confirmed.

## Serial Commands

- `ARM`: calibrate upright angle and enter balancing when calibration completes.
- `STOP`: stop motors and return to disarmed.
- `BALANCE`: leave drive mode but keep balancing.
- `DRIVE <forward> <turn>`: enable drive mode. Example: `DRIVE 20 0`.
- `PID <kp> <ki> <kd>`: update balance gains when runtime tuning is enabled.

## First Checks

1. Upload with wheels off the ground.
2. Open Serial Monitor at 115200 baud and set line ending to `Newline` or `Both NL & CR`.
3. Confirm debug output shows changing angle and distance values.
4. Send `ARM` while holding the robot upright and still.
5. Tilt the robot gently and confirm wheels correct in the direction that would drive under the falling body.
6. If either wheel runs backward, change `InvertRightMotor` or `InvertLeftMotor` in `config.h`.
7. If the balance angle does not change when tipping the robot forward/backward, change `BalanceGyroAxis` in `config.h`.
8. If both wheels correct the wrong way after the axis is correct, invert the sign of the selected gyro angle in `Sensors.cpp`.

## Balance Tuning

Start with small tests:

1. Keep `BalanceKi` at `0.0`.
2. Increase `BalanceKp` until the robot strongly corrects but does not oscillate violently.
3. Increase `BalanceKd` to damp oscillation.
4. Add a very small `BalanceKi` only if the robot consistently leans after proportional and derivative tuning.

## Obstacle Check

With the robot balancing, place an obstacle closer than the configured threshold and send `DRIVE 30 0`. Forward command should be clamped. `DRIVE -30 0` should still be allowed.
