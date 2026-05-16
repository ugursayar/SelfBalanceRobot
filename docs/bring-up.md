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
8. If both wheels correct the wrong way after the axis is correct, change `InvertBalanceOutput` in `config.h`.

## Balance Tuning

Start with small tests:

1. Keep `BalanceKi` at `0.0`.
2. If the robot shakes quickly, reduce `BalanceKp`, `BalanceKd`, or `MaxMotorCommand` in `config.h`.
3. If the robot falls without correcting strongly enough, increase `BalanceKp` in small steps and, only if the debug output shows motor commands near `MaxMotorCommand`, increase `MaxMotorCommand`.
4. Increase `BalanceKd` only enough to damp slow oscillation.
5. If correction starts strong but then rolls consistently in one direction, adjust `BalanceAngleTrimDegrees` by small amounts such as `0.5` or `-0.5`.
6. Add a very small `BalanceKi` only after proportional, derivative, motor limit, and trim are close.

## Obstacle Check

With the robot balancing, place an obstacle closer than the configured threshold and send `DRIVE 30 0`. Forward command should be clamped. `DRIVE -30 0` should still be allowed.
