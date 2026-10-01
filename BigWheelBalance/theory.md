# BigWheelBalance — why this algorithm, and where the gains come from

## 1. What changed and what it implies

| | old build | big-wheel build |
|---|---|---|
| wheel radius | 3.25 cm | 6.25 cm |
| motors | small encoder motors, slots 1/2 | 36 mm 12 V 240 rpm DC gear motors, no encoders |
| IMU height | low | 31 cm from the floor (~25 cm above the axle) |

The changes have four consequences:

- **Slower body.** The unstable pole is √(g/l_eq). The model puts it at **6.0 rad/s**, which doubles the lean in about 0.12 s. That is slower than the small robot, so it is more tolerant of delay and backlash, and 200 Hz is ample.
- **Wheel gains scale with radius.** A wheel radian now covers 1.9× the ground, so old gains can't be ported. They are re-derived from a model.
- **No encoders.** Wheel speed and travel are not measured. A plain tilt PD drives away whenever the balance point is a little off, because a steady lean δ needs a constant acceleration g·tanδ.
- **IMU height matters.** The accelerometer reads `ẍ + h·θ̈` as tilt (with h = 0.25 m, 1 m/s² reads as about 6°). The gyro is unaffected.

## 2. Algorithm choice

The options considered:

- **LQR (full-state feedback): chosen.**
  - It is the same structure as the proven `LqrBalance`.
  - It is equivalent to the classic cascade (inner angle PD, outer speed PI that sets the tilt setpoint), so you lose nothing by using it.
  - It costs about 0.1 ms of the 5 ms tick.
- **ADRC:** a possible later upgrade. Its observer lumps balance-point error and battery gain into one disturbance, but it adds two more tuning knobs, amplifies gyro noise, and doesn't stop drive-away on its own.
- **Sliding mode:** chatters. Abrupt PWM steps are exactly what excited backlash on the old build.
- **MPC:** too heavy for an 8-bit AVR. Embedded MPC solvers target 168–600 MHz Cortex-M parts. Unconstrained MPC *is* LQR anyway.

The design is built from the following pieces:

- **The plant input is voltage.** Torque per volt and back-EMF damping come from the motor datasheet, so the LQR gains come out in physical V/deg and V/(m/s). No "anchor to a hand-tuned gain" step is needed.
- **Missing wheel states come from a model observer.** See §3.
- **Discrete design at 200 Hz with one tick of delay.** The delay covers the IMU low-pass, the I2C read and the compute. The delay state adds the `kK_prevCmd` gain.
- **Balance-point trim.** It removes the residual arming error that the model-based travel estimate would otherwise hold against.

Research sources for the choice:

- B-ROBOT EVO2: cascade, setpoint from a speed PI.
- Elegoo Tumbller: TB6612 driver, PD plus a speed PI.
- YABR: setpoint integrator for encoder-less drift.
- NXTway-GS (Yamamoto 2008): voltage-driven wheeled-pendulum model with LQR.
- Laddach et al. 2023: a complementary filter with τ ≈ 0.6–1 s is as good as a Kalman filter for balancing robots.
- Chalmers thesis 2012: an LQG observer failed while plain PID worked, a caution against observers that rely on accelerometer integration.

## 3. Model and observer

This is the NXTway-GS form. The state is θ (mean absolute wheel angle) and ψ (body pitch, positive = leaning forward). The input v is the motor voltage, the same on both motors.

```
E [θ̈ ψ̈]ᵀ + F [θ̇ ψ̇]ᵀ + G [θ ψ]ᵀ = H v

E = [ (2m+M)R² + 2Jw + 2Jr     M·L·R − 2Jr      ]
    [ M·L·R − 2Jr              M·L² + Jψ + 2Jr  ]
F = 2·[ β  −β ; −β  β ]     β = Kt·Ke/Rm   (back-EMF damping, per motor)
G = [ 0 0 ; 0 −M·g·L ]
H = 2·[ α ; −α ]            α = Kt/Rm      (stall torque per volt, per motor)
```

Motor constants at the output shaft come from the datasheet:

- No load: 240 rpm at 12 V.
- Rated point: 182 rpm, 4 kg·cm, 1.2 A.

From those: Ke = 0.477 V·s/rad, Kt = 0.327 N·m/A, Rm = 2.42 Ω. The reflected rotor inertia Jr = n²·Jm ≈ 2.5e-3 kg·m² is an estimate.

**Observer.** Take the θ row and set z = θ̇ + c·ψ̇, with c = E₁₂/E₁₁. That eliminates ψ̈:

```
z'   = b·v − a·z + g·ψ̇          a = F₁₁/E₁₁,  b = H₁/E₁₁,  g = (F₁₁·c − F₁₂)/E₁₁
θ̇̂   = z − c·ψ̇                  travel = R·∫θ̇̂
```

Properties of this observer:

- It needs only the applied voltage and the gyro rate. No differentiation and no accelerometer.
- It is stable on its own: the pole at −a ≈ −9 rad/s.
- It includes the body's reaction on the wheels. A motor-only model would miss that, and it amounts to about 2 rad/s of error during a lean.
- Model error gives a **bounded** speed error. Travel can still drift; the trim and `kWheelPosLeakPerSec` handle that.

## 4. Design and checks (`design_gains.py`)

Bryson weights: 0.5 m travel, **5° tilt**, 0.5 m/s, **150°/s**, 6 V effort. For the estimated robot (2.0 kg body, CoM 0.14 m), they give:

| gain | value |
|---|---|
| tilt | 1.47 V/deg |
| tilt rate | 0.144 V/(deg/s) |
| travel | 10.9 V/m |
| speed | 29.7 V/(m/s) |
| previous command | −0.18 |

**Loop bandwidth matters more than the model suggests.**

- The first weights (3° tilt, 40°/s) put the loop crossover at **11 Hz**. On the robot that sustained a 10 Hz shiver near upright.
  - The tilt rate carried 8.9 °/s RMS at 6–15 Hz, and the command 0.94 V.
  - Gearbox backlash and frame flex add lag at 10 Hz that the model leaves out. At small amplitudes that eats the margin.
- The current weights put the crossover at **5.6 Hz** (75° phase margin) and cut the loop gain at 10 Hz from 1.12 to 0.55. The shiver dropped by about 70%.
- The cost: a slower ~2 Hz backlash sway, about ±1.2°. A 0.15 V deadband compensation halved it in the log (run C). On the robot, though, the user preferred no compensation (run B) once driving at 0.8 m/s.
- `design_gains.py` prints the crossover and |L| at 10 Hz. Keep them at about 4–6 Hz and below 0.6.

**Robustness.** The closed loop includes the true plant, the delay and the nominal observer. It is stable across body mass from 0.6× to 1.5× of nominal and CoM height from 0.6× to 1.6× at nominal rotor inertia and battery. Two corners fail:

- A low CoM (0.6×, or 0.75× on the lighter bodies) combined with low rotor inertia and a full pack.
- A very tall, heavy robot combined with high rotor inertia and a low pack.

So measure mass and CoM, and re-run the design.

**Nonlinear simulations** cover the tilt filter fed by a motion-corrupted accelerometer at 25 cm, gyro and accelerometer noise, slew and clamp:

| case | result |
|---|---|
| 5° and 10° starts | held, max travel 5–11 cm |
| pack ±15% | held |
| 12 N push for 0.1 s | held, 32 cm travel |
| 1 V motor deadband | held, but a ±1.9° limit cycle (the "wobble") |
| 1 V deadband with 1 V compensation | wobble drops to ±0.3° |
| 3° arming error, no trim | drives off about 3 m |
| 3° arming error, trim τ = 4 s | returns to the arm spot (peak 50 cm) |

## 5. Tilt estimate

- Complementary filter with τ = 0.8 s, gyro-dominant. In simulation, 0.5–1.5 s all held.
- The gyro bias is averaged over 1 s at boot.
- The accelerometer is ignored for any tick where |a| is more than 0.25 g away from 1 g.
- The MPU6050 low-pass is set to 94/98 Hz (about 3 ms of delay), inside the one-tick delay budget.
- Motion compensation (subtracting `ẍ + h·θ̈` from the accelerometer) is a possible later refinement. The long τ already averages the error out in simulation.
