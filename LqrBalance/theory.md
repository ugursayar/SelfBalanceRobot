# LqrBalance — deriving the gain row `K`

This is the companion to [LqrBalance.ino](LqrBalance.ino). It gives the linearized
plant model, a runnable Python script that solves the LQR, and the exact mapping
from the textbook gains into the sketch's four `kK_*` constants.

The sketch ships with **seed** gains (tilt/tilt-rate from the known-good hand
tuning, small clamped wheel gains). Those are stable but not optimal. Use this
doc when you want a model-derived `K`.

---

## 1. State, units, and sign convention

The sketch's state vector, in its own order and units:

| # | state      | sketch variable        | unit    |
|---|------------|------------------------|---------|
| 1 | tilt       | `tiltErr`              | **deg** |
| 2 | tilt rate  | `tiltRate`             | **deg/s** |
| 3 | wheel pos  | `wheelPosDeg`          | **deg** (avg wheel angle from arm point) |
| 4 | wheel vel  | `gWheelVelFiltDegPerSec` | **deg/s** |

Control output `u` is a motor **PWM** command (−180…180), and **positive `u`
recovers a forward lean**.

LQR is naturally derived in SI units (radians, metres, newtons), so there is a
unit + scale conversion at the end. The honest reality on a hobby drivetrain:
**LQR gives you the right relative gains and structure; the absolute scale and
the wheel-term signs get pinned empirically on your robot** (the PWM→torque
constant, deadband, and encoder mounting sign are not known precisely). The
script below handles the unit conversion and anchors the overall scale to your
working tilt gain so you never need the motor constant.

---

## 2. Linearized plant (wheeled inverted pendulum ≈ cart-pole)

Planar balance axis only; yaw, wheel slip, and motor electrical dynamics ignored.
Linearized about upright, in the sketch's state order **[θ, θ̇, x, ẋ]** (θ = tilt
in rad, x = base position in m, input = equivalent horizontal force `F` in N):

```
        ⎡ 0                       1        0   0                    ⎤
A   =   ⎢ m·g·l·(M+m)/p           0        0  -m·l·b/p              ⎥
        ⎢ 0                       0        0   1                    ⎥
        ⎣ m²·g·l²/p               0        0  -(I+m·l²)·b/p         ⎦

        ⎡ 0            ⎤
B   =   ⎢ m·l/p        ⎥
        ⎢ 0            ⎥
        ⎣ (I+m·l²)/p   ⎦

p   =   I·(M+m) + M·m·l²
```

Parameters (measure each on your robot):

| sym | meaning | how to get it | rough start |
|-----|---------|---------------|-------------|
| `M` | base + wheels mass (kg) | scale | 0.5 |
| `m` | pendulum body mass (kg) | scale | 0.5 |
| `l` | axle → body CoM distance (m) | balance the body on a finger | 0.07 |
| `I` | body inertia about its CoM (kg·m²) | ≈ `m·l²/3` for a rod-like body | `m*l*l/3` |
| `r` | wheel radius (m) | calipers | 0.0325 |
| `b` | base viscous damping (N·s/m) | usually negligible | 0.0 |
| `g` | gravity | — | 9.81 |

The base position `x` and the sketch's wheel **angle** are related by `x = r·φ`
(φ = wheel angle in rad), which is folded into the mapping in §4.

---

## 3. Solve the LQR (Python)

Dependencies: `numpy`, `scipy` (`pip install numpy scipy`). No `control` package
needed — this uses the continuous-time algebraic Riccati solver directly.

```python
import numpy as np
from scipy.linalg import solve_continuous_are

# ---- 1. plant parameters (EDIT THESE) -------------------------------------
M, m, l, I, r, b, g = 0.5, 0.5, 0.07, 0.5*0.07**2/3, 0.0325, 0.0, 9.81

p = I*(M + m) + M*m*l**2
A = np.array([
    [0.0,                 1.0, 0.0, 0.0],
    [m*g*l*(M + m)/p,      0.0, 0.0, -m*l*b/p],
    [0.0,                 0.0, 0.0, 1.0],
    [m**2 * g * l**2 / p,  0.0, 0.0, -(I + m*l**2)*b/p],
])
B = np.array([[0.0], [m*l/p], [0.0], [(I + m*l**2)/p]])

# ---- 2. weights: Bryson's rule  Q_ii = 1/max_i^2 ,  R = 1/max_u^2 ----------
# "max" = the largest deviation/effort you are willing to tolerate.
max_tilt_rad   = np.radians(8.0)    # allow ~8 deg tilt
max_tiltrate   = np.radians(60.0)   # ~60 deg/s
max_pos_m      = 0.10               # ~10 cm drift
max_vel_ms     = 0.30               # ~0.3 m/s
max_force_N    = 4.0                # effort ceiling

Q = np.diag([1/max_tilt_rad**2, 1/max_tiltrate**2, 1/max_pos_m**2, 1/max_vel_ms**2])
R = np.array([[1/max_force_N**2]])

# ---- 3. solve  u = -K x ----------------------------------------------------
P = solve_continuous_are(A, B, Q, R)
K = (np.linalg.inv(R) @ B.T @ P).flatten()   # [Kθ, Kθ̇, Kx, Kẋ]  (SI)
print("K_si =", K)

# ---- 4. convert to the sketch's deg / wheel-deg units ----------------------
d2r = np.pi/180.0
shape = np.array([K[0]*d2r,          # per deg tilt
                  K[1]*d2r,          # per deg/s tilt rate
                  K[2]*r*d2r,        # per deg wheel angle   (x = r*phi)
                  K[3]*r*d2r])       # per deg/s wheel rate

# ---- 5. anchor the overall scale to your working tilt gain -----------------
# This removes the unknown PWM-per-Newton constant: keep the LQR *ratios*, but
# scale the whole row so the tilt gain equals what already balances your robot.
WORKING_TILT_GAIN = 28.0
alpha = WORKING_TILT_GAIN / shape[0]
kK = alpha * shape
print(f"kK_tilt     = {kK[0]:.3f}f")
print(f"kK_tiltRate = {kK[1]:.3f}f")
print(f"kK_wheelPos = {kK[2]:.4f}f   # sign: bench-verify")
print(f"kK_wheelVel = {kK[3]:.4f}f   # sign: bench-verify")
```

---

## 4. Mapping to the sketch + signs

The script prints the four numbers in the sketch's units. Paste their
**magnitudes** into [LqrBalance.ino](LqrBalance.ino):

```c
static const float kK_tilt     = ...;   // positive  (recovers a forward lean)
static const float kK_tiltRate = ...;   // positive
static const float kK_wheelPos = ...;   // see sign note below
static const float kK_wheelVel = ...;   // see sign note below
```

> **Bench finding on this robot:** the model's wheel gains come out **negative**,
> but the encoder/motor wiring here is **flipped from the model**, so the
> bench-verified signs are **POSITIVE** (`kK_wheelVel = +`, confirmed by the
> spin test). A model-derived `K` therefore needs both wheel gains' signs
> **flipped to positive** before flashing. Also note: the position term tends to
> **ring up** on this robot regardless of magnitude (backlash), so station-keeping
> has stayed unreliable — see [CLAUDE.md](CLAUDE.md).

**Why anchoring works:** LQR's value here is the *ratio* of the gains; the
absolute scale depends on the PWM→torque relationship, which you don't know.
Anchoring `kK_tilt` to the value that already balances your robot (≈28) transfers
that calibration to the other three gains automatically.

**Signs:** tilt and tilt-rate are **positive**. The wheel terms are **positive on
this robot** (bench-verified — opposite to the model, because the wiring is
flipped), so flip the model's negative wheel gains to positive before flashing,
then confirm with the two bench checks from the sketch header:

1. **Wheel velocity** — spin a wheel by hand while balancing; it should *resist*.
   Runs away → flip `kK_wheelVel`.
2. **Wheel position** — nudge the balanced robot forward; it should creep back to
   where it started. Accelerates away → flip `kK_wheelPos`.

The encoder mounting sign was never validated in the original firmware (those
gains were always 0), so do not skip these checks. The sketch's wheel-term clamp
(±40) plus the 35° fall-cutoff keep a wrong sign recoverable.

---

## 5. Practical workflow

1. Measure `M, m, l, r`; estimate `I ≈ m·l²/3`.
2. Run the script; start with the Bryson weights above.
3. Paste the four magnitudes; keep the documented signs.
4. Upload, stand it up, run the two bench checks; flip wheel signs if needed.
5. Tune by feel through `Q`/`R`, not the gains directly:
   - shakier/over-eager near upright → **lower** `Q[tilt]` or **raise** `R`.
   - drifts / won't hold position → **raise** `Q[pos]`.
   - wanders at speed → **raise** `Q[vel]`.

### Limitations

This planar model ignores motor electrical dynamics, the PWM deadband/nonlinearity,
wheel slip, and two-wheel yaw. It gets you a well-conditioned starting `K`; the
last bit is always bench tuning — which is why the sketch keeps every gain as a
single named constant at the top.
