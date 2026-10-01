# BigWheelBalance — implementation notes

Standalone balance sketch for the big-wheel build:

- 36 mm DC gear motors on the MegaPi DC terminals, **no encoders**.
- 12.5 cm wheels.
- IMU 31 cm above the floor.

Keep the hot loop serial-free, as in `LqrBalance`. `../BigWheelBringUp` is the serial-based hardware check.

## Hardware traps

- **Port names.** The MegaPi DC terminals are `PORT1A..PORT4A` (B = the encoder-slot channel, `PORT1B..4B`). The library has no "5A/6A", although the user described the motors as on "Port 5A / 6A".
  - Bench-confirmed on 2026-10-01: **left = PORT1A, right = PORT2A**.
  - +PWM rolls the left wheel forward and the right wheel backward, so the signs are left +1, right −1.
- **IMU orientation is flipped vs MeGyro's `angleX`.** A forward lean drives accel X negative and gyro Y positive (`kAccelFwdSign = -1`, `kGyroPitchSign = +1`).
  - The chip reports WHO_AM_I 0x98 (MPU6050-compatible clone).
  - Gyro Y has about +3 °/s of bias, removed by the boot calibration.
  - Opening the serial port resets the board, even through .NET `SerialPort` with DTR off. Build `BigWheelBringUp` with `-DBRINGUP_IMU_ONLY` to stream without re-running the motors.
- **The DC terminals need the plug-in driver modules.** The MegaPi main board has no H-bridges. Each slot's module drives both its B channel (the encoder port on the module) and its A channel (the DC screw terminals on the main board below it).
  - Removing the slot 1/2 modules silently kills PORT1A/2A.
  - It looked like a firmware bug: armed, LED solid, no motion. The model observer, fed volts that move nothing, then winds the command to the rail.
  - The stall cutoff now trips that within 1.5 s.
  - Leg encoder motors should get their own modules in slots 3/4, not share slots 1/2.
- **Makeblock shadows `Wire.h`.** `MeMegaPi.h` → `MeConfig.h` includes Makeblock's old `utility/Wire.h`, which hides the core's `setWireTimeout()`.
  - The linked implementation is the core's (AVR 1.8.8), so the sketch calls `twi_setTimeoutInMicros()` through an `extern "C"` declaration.
  - Don't `#include <Wire.h>` next to `MeMegaPi.h`. It doesn't help, and it mixes the two declarations.
- **PWM timers must be set in `setup()`.** `MeMegaPiDCMotor`'s constructor sets the timers, but globals are constructed before the core's `init()`, which resets them. So `configureDcMotorPwmTimers()` re-applies 976 Hz on Timers 1–4. Timer 0 (pin 4, `PORT4A`) also drives `millis()`; never touch it.
- **`run(0)` coasts** (IN1 = IN2 = LOW). A nonzero command is drive/brake PWM on a TB6612-class bridge, so average voltage ≈ duty × Vbatt. That is the linearity the volts-based design assumes.
- **IMU is read directly**: MPU6050 at 0x68, DLPF 2, ±500 °/s, ±4 g, 400 kHz.
  - `readImu()` reads high and low bytes in separate statements. The operand evaluation order of `(read() << 8) | read()` is unspecified in C++.
  - Axis and sign constants come from the bring-up stream. Defaults copy MeGyro's `angleX` (accel X, gyro Y negated).
- **Model the H-bridge resistance.** `design_gains.py` uses the motor's 2.42 Ω plus `R_DRIVER` = 1 Ω (TB6612-class on-resistance, two switches in the path).
  - Without it the wheel observer over-predicted speed per volt.
  - That gave a 3–4 Hz mode, damping about 0.35 with 15% sag: the full-throttle wobble that grew with speed. The black-box log found it.
  - With it, the damping is about 0.8 and the wobble is gone on the robot.
  - Neither speed-only driving nor the S-curve touched it.
- **Driver current.** The motor stall is about 5 A, and the MegaPi driver is about 1 A continuous / 2 A peak. If the robot goes limp on hard recoveries, suspect the driver before the gains.

- **Bluetooth controller** (official Makeblock gamepad → Me Bluetooth module on `Serial3`, 115200).
  - It streams MePS2-format packets at about 80/s: `FF 55 LX btn LY btn RX btn RY sum`. The sum is the 8-bit sum of the 7 payload bytes.
  - Sticks: centre 128, up/left = 0. Buttons are bitfields; triangle = byte 3 bit 0, square = bit 2.
  - Bench-verified with `BigWheelBringUp -DBRINGUP_BT`, which also showed that a left (CCW) turn reads gyro Z **positive**.
  - Parse it every `loop()` pass, not only on control ticks; Serial3's RX buffer is 64 bytes.

## Control decisions

- **Volts, not PWM.** Every gain is in volts, converted once with `kBatteryVolts`, so the model gains apply directly and a battery change is one constant. This is the old build's "battery is a hidden gain" lesson, made explicit.
- **No encoders: model observer.** Wheel speed comes from the θ row of the voltage-driven pendulum model. It is driven only by the applied voltage and the gyro rate (see theory.md §3).
  - The observer takes the voltage applied over the tick that just ended (`gAppliedVolts` before it is overwritten). Keep that ordering.
  - It takes the **linear** command; deadband compensation is added after it, on the assumption that it cancels the motor's static friction.
- **One-tick delay is part of the design.** `kK_prevCmd` is negative by design. It comes from the delay-augmented LQR and improves robustness for low-inertia or high-battery cases. Don't drop it without re-checking the robustness map.
- **Balance-point trim.** Integrates the position term into the setpoint, with τ = 4 s and a cap of ±6° from the arm tilt.
  - Without it, a d° arming error parks the robot about 21 cm per degree away from its start. Past about 2° the position clamp saturates and it drives away (simulated: 3° → 3 m).
  - With it, the robot returns to its arm spot.
- **Loop crossover is the near-upright quality knob.** This drivetrain has backlash and a flexible frame, so it forms small limit cycles at whatever frequency the loop crosses over.
  - At 11 Hz (first weights) it was a 10 Hz shiver.
  - At 5.6 Hz (current: tilt 5°, rate 150°/s) it is a slower ~2 Hz sway, which the user called calmer.
  - Bench-measured, not guessed: debug build at 100 Hz, then FFT.
  - Don't push the crossover back up, and keep |L| at 10 Hz below about 0.6 (the design script prints both).
- **Shaping kept minimal.** Clamp, then slew limit (1.5 V/tick ≈ the proven 30 PWM), then smooth deadband compensation.
  - Compensation is **0** (config "B").
    - 0.15 V (config "C") halved the 2 Hz sway in the log.
    - Once driving at 0.8 m/s, the user preferred B: it stands calmer, with a bit more lean but less movement, and it wobbles less at full throttle.
    - 0.3 V on the 11 Hz loop made the shiver worse, because the compensation raises small-signal gain.
  - No curved power, no coast band: pure LQR was the best base on the old build.
- **Still-check calibration.** The boot calibration only accepts a 1 s window in which the pitch rate, yaw rate and accel tilt stay within 10 °/s and 3° peak-to-peak; otherwise it retries, toggling the LED.
  - A reset while balancing once calibrated mid-fall: a 27 °/s false bias drove the command to the rail, and the stall cutoff caught it.
- **Driving = speed reference, not a tilt offset.**
  - The stick ramps `gSpeedRef` at 0.5 m/s², up to 0.8 m/s.
  - The LQR runs on the speed error plus a back-EMF feedforward `Ke/r · v_ref`. At a steady speed the model needs that voltage and zero lean.
  - **While driving, `gTravelRef` follows the robot (speed-only).** Position hold resumes where the ramp reaches zero.
    - Integrating the reference instead made the robot catch up after lagging it, overshooting about 16% in the sim.
    - On the robot the full-throttle wobble felt the same either way.
  - The delay-comp term uses the deviation `u_prev − ff_prev`.
  - The trim is frozen while driving.
  - **Tried and dropped (2026-10-02): S-curve plus acceleration feedforward.** This was a jerk-limited profile (2 m/s³) with the model's lean of 8.9° per m/s² and 0.84 V per m/s² fed forward.
    - The sim halved the corner and stop rocking.
    - On the robot it was worse in every way. Re-flashing B restored good behaviour.
    - Don't re-add it without bench data. The model's acceleration lean, which is dominated by the uncertain reflected rotor inertia, is the prime suspect.
  - Full-throttle wobble remains (user: "fires more power and leans back").
  - Turning adds ±diff volts per wheel (right +, left −) from a yaw-rate loop on gyro Z. The diff is clamped to the headroom left after the balance command, so balance always wins.
- **Stall cutoff.** If |u| ≥ 11 V for 1.5 s, the robot disarms and locks out until the tilt passes the fall angle. Real recoveries saturate for well under 0.5 s; a stuck robot or dead motors pin it.
- **Yaw loop = rate P + heading hold.** The rate P alone (`kTurnP` 0.02 V per °/s) was too soft to feel against a hand twist, so the heading hold was added: `kHeadingP` 0.10 V/deg on the integral of the rate error, capped at ±30°.
  - Yaw plant estimate: about 68 °/s per V with about 70 ms lag.
  - Simulated: no overshoot over 0.5–2× gain and 0.03–0.2 s lag.
  - The stick moves the held heading, because the integral runs on (turnRef − rate).
  - Residual gyro-Z bias makes the held heading creep, about 0.1 °/s at most.
- **Black box.** A `.noinit` ring buffer (480 samples × 5 int16 at 50 Hz, 9.6 s) survives the USB-open reset and is dumped at boot when the magic and running checksum check out.
  - Freeze triggers: any button (packet bytes 1/3/5 non-zero, kept until dumped; the armed LED blinks) and disarm (a re-arm overwrites it).
  - A triangle-only freeze never registered on the robot: the pad's labels don't match the library's PS2 names.
  - It costs about 4.8 KB of RAM (75% used, 2 KB stack left). Shrink `kBlackBoxSamples` before adding big globals.
  - Read it with pyserial, which resets the board; the dump comes before calibration.
- **Debug build:** `--build-property "compiler.cpp.extra_flags=-DBWB_DEBUG"` logs at 100 Hz on USB, as integers: tilt error [c°], u [cV], rate [0.1 °/s], PWM R, PWM L. Never ship it.
  - **Every port open resets the board.** That includes pyserial with dtr/rts off; it's the CH340 auto-reset.
  - So the robot must be *held still* through both the upload reset and the capture reset, then released after the second LED-solid.
  - A 2.5 s pause between upload and opening the port lets the first calibration finish.
- **Gains are a matched set** with the observer constants. Regenerate them with `design_gains.py`; don't hand-mix.

## Testing

- No native tests. The sketch is hardware-coupled.
- `design_gains.py --sim` is the check: a nonlinear plant with an accelerometer corrupted by motion at the 25 cm sensor height, deadband, battery error, and pushes.
- Compile both sketches before flashing.
