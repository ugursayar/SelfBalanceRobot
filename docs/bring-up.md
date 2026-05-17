# Bring-Up Guide

Use short tests and keep the robot held securely until motor direction and balance correction direction are confirmed.

## Serial Commands

- `ARM`: calibrate upright angle and enter balancing when calibration completes.
- `STOP`: stop motors and return to disarmed.
- `BALANCE`: leave drive mode but keep balancing.
- `DRIVE <forward> <turn>`: enable drive mode. Example: `DRIVE 20 0`.
- `PID <kp> <ki> <kd>`: update balance gains when runtime tuning is enabled.

Commands are case-insensitive, so `arm`, `drive 20 0`, and `pid 35 0 0.9` also work.

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
2. If the robot shakes only near upright, reduce `SmallErrorGainScale`, reduce `BalanceKd`, or increase `SmallErrorDegrees` in `config.h`.
3. If the robot shakes quickly at larger lean angles, reduce `BalanceKp`, `BalanceKd`, or `MaxMotorCommand`.
4. If the robot falls without correcting strongly enough, increase `BalanceKp` in small steps and, only if the debug output shows motor commands near `MaxMotorCommand`, increase `MaxMotorCommand`.
5. Increase `BalanceKd` only enough to damp slow oscillation.
6. If correction starts strong but then rolls consistently in one direction, adjust `BalanceAngleTrimDegrees` by small amounts such as `0.5` or `-0.5`.
7. Add a very small `BalanceKi` only after proportional, derivative, motor limit, and trim are close.

You can tune gains without re-uploading by sending a `PID` command from Serial Monitor. Examples:

- `pid 25 0 0.7`
- `pid 35 0 0.9`
- `pid 45 0 1.1`
- `pid 60 0 1.4`

If debug `left=` and `right=` are near `MaxMotorCommand` while the robot still cannot recover, increase `MaxMotorCommand`. If they are not near the limit, increase `BalanceKp` first.

If debug `balance=` changes but `left=` and `right=` stay at zero, software deadband is too high. Keep `MotorDeadband` low or `0` while tuning balance.

## Obstacle Check

With the robot balancing, place an obstacle closer than the configured threshold and send `DRIVE 30 0`. Forward command should be clamped. `DRIVE -30 0` should still be allowed.
