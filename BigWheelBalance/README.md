# BigWheelBalance

Balance firmware for the **big-wheel build**:

- Makeblock 36 mm 12 V / 240 rpm DC gear motors on the MegaPi DC terminals. These motors have **no encoders**.
- 12.5 cm wheels.
- MPU6050 31 cm above the floor (about 25 cm above the axle).

Like `LqrBalance`, it is a standalone sketch with no serial output in the control loop. The old builds in `../LqrBalance` and `../SelfBalanceRobot` are untouched.

Companion files:

- [`../BigWheelBringUp`](../BigWheelBringUp): a one-shot hardware check. **Run it first.**
- [`design_gains.py`](design_gains.py): the model, the gain design and the simulations that produce every gain in the sketch.
- [`theory.md`](theory.md): why this algorithm, and how the gains are derived.

## Algorithm (short version)

The controller is a discrete LQR at 200 Hz that computes its command in **volts**. Without encoders, wheel speed and travel come from a **model-based observer**. The motor is voltage-driven, so its back-EMF ties wheel speed to the body dynamics. The observer estimates wheel speed from the voltage it applied plus the gyro rate, and integrates that speed to get travel.

On top of that:

- A slow **balance-point trim** moves the tilt setpoint onto the true balance point, so the robot doesn't need to lean back from its arm spot.
- A **slew limit** smooths command steps (it was the key no-wobble fix on the old drivetrain).
- Optional **smooth deadband compensation** for motor static friction.

```
u[V] = kK_tilt*tiltErr + kK_tiltRate*tiltRate + kK_wheelPos*travelHat
     + kK_wheelVel*speedHat + kK_prevCmd*uPrev
```

## Before the first run

1. **Flash `BigWheelBringUp`** and open the serial monitor at 115200 with the wheels off the ground.
   - It spins each DC port (PORT1A..4A, then 1B..4B) in turn. Note which wheel turns on which step, and which way the robot would roll.
   - Then it streams the IMU. Stand the robot upright, lean it forward, then back.
   - From that, set `kRightMotorPort`/`kLeftMotorPort`, the two motor signs, and the IMU axis map at the top of `BigWheelBalance.ino`.
   - Positive command must roll **both** wheels "forward", and leaning forward must make both tilt and tilt rate positive.
2. **Measure and re-run the gain design.** The shipped gains assume a body of 2.0 kg (everything except the wheels) and a CoM 14 cm above the axle; both are estimates.
   - Weigh the robot.
   - Find the CoM: lay the robot horizontally on a rod and slide it until it balances. The CoM height is the distance from the axle to that point.
   - Edit `M_BODY` and `L_COM` in `design_gains.py`, run `python design_gains.py --sim`, and paste the printed block.
3. **Set `kBatteryVolts`** to the pack voltage. The command is in volts, so this keeps the loop gain constant across charge levels.

## Operation

Same feel as `LqrBalance`:

1. Hold the robot still at its balance point through power-up (about 1.2 s; gyro bias and the balance point are latched then).
2. The LED blinks faster as you near the balance point. It arms inside ±4° while steady, and the LED goes solid.
3. Past 35° it disarms and the motors coast.

- If the boot tilt is beyond 15° (e.g. lying down), the 0° fallback is used, so it never arms lying down.
- A fast continuous blink means the IMU did not answer at boot.

## Driving (Makeblock Bluetooth controller)

Pair the official Makeblock Bluetooth controller with the Me Bluetooth module, which is on `Serial3` at 115200. Then drive:

- **Left stick up/down:** forward/back, up to `kMaxDriveSpeedMps` (0.8 m/s). The speed ramps at `kDriveAccelMps2` (0.5 m/s²). While moving it controls speed only, then holds the spot where it comes to rest.
- **Right stick left/right:** turn left/right, up to `kMaxTurnRateDps` (90 °/s).

How the controller handles it:

- The stick sets a speed reference. The balance loop leans into it and stops on the spot where you release the stick.
- Turning is a gyro yaw loop with **heading hold**. The robot drives straight, pushes back when twisted, and returns to its heading. That memory is capped at ±30°, so anything twisted beyond that is forgotten.
- If no packet arrives for 300 ms (controller off or out of range), the sticks count as centred and the robot ramps to a stop.
- The balance-point trim is frozen while driving.
- Set `kEnableBluetoothDrive = false` for the balance-only build.

Simulated at 0.8 m/s: 7.1 V peak, which leaves about 5 V for braking and recovery. 1.2 m/s would approach the 11 V stall cutoff.

Drive levers:

| Want | Lever |
|---|---|
| faster / slower | `kMaxDriveSpeedMps` (keep ≤ 0.8: back-EMF eats balance headroom) |
| gentler starts and stops | lower `kDriveAccelMps2` |
| turns too slow / too fast | `kMaxTurnRateDps`; a lazy or overshooting turn → `kTurnP` |
| twist push-back too soft / too stiff | `kHeadingP` (0.10 V/deg) |
| keeps creeping after stop | `kDriveFeedforwardVoltsPerMps` (motor friction) |

## First-run checks (hold it, be ready to catch)

- **Direction:** armed and held, tip it forward. Both wheels must drive forward, under the lean.
  - Both drive the wrong way: flip both motor signs.
  - Only one is wrong: flip that one motor's sign.
- **Return to spot:** push it gently. It should roll, recover, and drift back toward where it armed over a few seconds.

## Tuning levers (one at a time)

| Symptom | Lever |
|---|---|
| Slow ~2 Hz sway near upright (gearbox play / sticking) | `kDeadbandCompVolts` (now 0): 0.15 V halved it but the robot drove worse; 0.3 V brought the 10 Hz shiver back |
| Fast ~10 Hz shiver | loop too fast for the gearbox: re-run the design with gentler weights (larger `MAX_TILT_DEG` / `MAX_TILT_RATE_DPS`) until the printed crossover is ~4–6 Hz |
| Soft, sags or falls on pushes | raise `kMaxVoltStepPerTick`; re-run the design with smaller `MAX_TILT_DEG` |
| Slow wandering back and forth | `kBalanceTrimTauSec` larger (slower trim) or `kWheelPosLeakPerSec` > 0 |
| Drives off steadily | check `kBatteryVolts` and the trim is on; raise `kWheelPosClampVolts` |
| Behaviour changes with charge | update `kBatteryVolts` |

Prefer re-running `design_gains.py` with new weights or measurements over hand-editing the K row. The gains are a matched set with the observer constants.

## Diagnostics

The release build has no logging or USB output; it is silent apart from the Bluetooth controller link.

Two tools were used to tune it and can be restored from commit `1198d6f`:
- a 100 Hz USB debug log (`-DBWB_DEBUG`);
- a black-box logger: an in-RAM ring buffer of the last 9.6 s, frozen by any controller button and dumped on USB at the next boot.

## Stall cutoff

If the command stays at or above 11 V for 1.5 s, the motors cut out. Causes: the robot is stuck against something, or the motors are not responding (e.g. a driver module is missing; the DC terminals only work with the slot's module plugged in).

To re-arm: lay the robot down (past 35°), then stand it back up at the balance point.

## Hardware risk: driver current

The 36 mm motor stalls at roughly 5 A (estimated from the datasheet). The MegaPi's plug-in motor driver is rated around 1 A continuous / 2 A peak per channel. Hard recoveries will push it.

If the robot goes limp mid-recovery, or the driver chips get hot, the driver is limiting. Lower `kMaxVolts`, or move to a higher-current driver.

## Build & upload

```
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 BigWheelBalance
arduino-cli upload  -p <PORT> --fqbn arduino:avr:mega:cpu=atmega2560 BigWheelBalance
```
