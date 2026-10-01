"""
BigWheelBalance gain + observer designer.

Edit the PARAMETERS block with your measured values, run

    python design_gains.py            # gains + observer constants + checks
    python design_gains.py --sim      # also run the nonlinear simulations

and paste the printed C block into BigWheelBalance.ino.

Model: wheeled inverted pendulum driven by VOLTAGE through a DC gear motor
(NXTway-GS form, Yamamoto 2008), linearized about upright.  State
x = [theta, psi, theta_dot, psi_dot]: theta = mean wheel angle (rad, absolute),
psi = body pitch (rad, + = leaning forward), input v = motor voltage (V, same on
both motors, + drives forward).  The controller is designed in discrete time at
the 200 Hz loop rate with one tick of input delay (IMU low-pass + compute), which
adds a "previous command" gain.  Requires numpy + scipy.
"""
import sys

import numpy as np
from scipy.linalg import expm, solve_discrete_are

# ============================== PARAMETERS ==================================
# Robot (MEASURE these: kitchen scale + balance the robot horizontally on a
# rod to find the CoM height above the axle).  Defaults are estimates.
M_BODY = 2.0      # kg   everything except the two wheels (motors included)
M_WHEEL = 0.12    # kg   one wheel incl. hub
L_COM = 0.14      # m    axle -> body centre of mass
J_BODY = None     # kg m^2 body pitch inertia about its CoM (None = M L^2 / 3)
R_WHEEL = 0.0625  # m    wheel radius (12.5 cm diameter)

# Motor: Makeblock 36 mm DC geared motor 12 V 240 rpm (datasheet: 182 rpm and
# 4 kg.cm at 1.2 A rated).  Constants are referred to the OUTPUT shaft.
V_NOMINAL = 12.0                                  # V  battery the PWM is scaled to
KE = V_NOMINAL / (240.0 * 2.0 * np.pi / 60.0)     # V/(rad/s)  back-EMF   ~0.477
KT = 0.392 / 1.2                                  # N m/A      effective  ~0.327
R_MOTOR = (V_NOMINAL - KE * 182.0 * 2.0 * np.pi / 60.0) / 1.2   # ohm  ~2.42
J_ROTOR = 2.5e-3  # kg m^2 rotor inertia reflected through the gearbox (n^2 Jm), estimate
# The H-bridge's on-resistance sits in series with the winding (TB6612-class:
# ~0.5 ohm per switch, two in the current path).  Leaving it out made the wheel
# observer expect more push per volt than the robot delivers: a 3-4 Hz mode,
# lightly damped and worse with battery sag, showed up as the full-throttle
# wobble (black-box log 2026-10-02).  Modelled, the mode's damping goes
# 0.35 -> 0.81 at 15% sag.
R_DRIVER = 1.0    # ohm
R_TOTAL = R_MOTOR + R_DRIVER

TS = 0.005        # s   control period (200 Hz)

# LQR weights, Bryson's rule: the largest excursion you will tolerate per state.
# The first set (tilt 3 deg, rate 40 deg/s) put the loop crossover at 11 Hz;
# on the robot that showed up as a 10 Hz shiver near upright (gearbox backlash
# and frame flex add lag the model lacks).  These weights put the crossover at
# ~5.6 Hz with the loop gain at 10 Hz below 0.6 -- keep it there (see the
# "loop" line this script prints).
MAX_TRAVEL_M = 0.5          # wheel travel from the arm point
MAX_TILT_DEG = 5.0          # body tilt
MAX_SPEED_MPS = 0.5         # ground speed
MAX_TILT_RATE_DPS = 150.0   # body tilt rate
MAX_VOLTS = 6.0             # control effort (half the pack leaves headroom)
# ============================================================================


def plant(M=M_BODY, m=M_WHEEL, L=L_COM, Jr=J_ROTOR, Rm=R_TOTAL, R=R_WHEEL,
          Jpsi=J_BODY, Ke=KE, Kt=KT, g=9.81):
    """Continuous model. Returns A, B and the theta-row terms the observer uses."""
    if Jpsi is None:
        Jpsi = M * L * L / 3.0
    Jw = m * R * R / 2.0
    alpha = Kt / Rm            # stall torque per volt (one motor)
    beta = Kt * Ke / Rm        # back-EMF damping (one motor)
    E = np.array([[(2 * m + M) * R * R + 2 * Jw + 2 * Jr, M * L * R - 2 * Jr],
                  [M * L * R - 2 * Jr, M * L * L + Jpsi + 2 * Jr]])
    F = 2.0 * np.array([[beta, -beta], [-beta, beta]])
    G = np.array([[0.0, 0.0], [0.0, -M * g * L]])
    H = 2.0 * np.array([[alpha], [-alpha]])
    Ei = np.linalg.inv(E)
    A = np.zeros((4, 4))
    A[0, 2] = A[1, 3] = 1.0
    A[2:, :2] = -Ei @ G
    A[2:, 2:] = -Ei @ F
    B = np.zeros((4, 1))
    B[2:, :] = Ei @ H
    return A, B, (E, F, H)


def c2d(A, B):
    n = A.shape[0]
    Mx = np.zeros((n + 1, n + 1))
    Mx[:n, :n], Mx[:n, n:] = A, B
    Ed = expm(Mx * TS)
    return Ed[:n, :n], Ed[:n, n:]


def design():
    A, B, _ = plant()
    Ad, Bd = c2d(A, B)
    Aa = np.block([[Ad, Bd], [np.zeros((1, 5))]])        # + one-tick input delay
    Ba = np.vstack([np.zeros((4, 1)), [[1.0]]])
    R = R_WHEEL
    Q = np.zeros((5, 5))
    Q[:4, :4] = np.diag([1 / (MAX_TRAVEL_M / R) ** 2, 1 / np.radians(MAX_TILT_DEG) ** 2,
                         1 / (MAX_SPEED_MPS / R) ** 2, 1 / np.radians(MAX_TILT_RATE_DPS) ** 2])
    Rw = np.array([[1 / MAX_VOLTS ** 2]])
    P = solve_discrete_are(Aa, Ba, Q, Rw)
    return np.linalg.solve(Rw + Ba.T @ P @ Ba, Ba.T @ P @ Aa).flatten()  # u = -K [x; u_prev]


def observer_coeffs():
    """Wheel-speed observer from the theta row of the model (no encoders):
         z' = b*v - a*z + g*psi_dot ,  theta_dot_hat = z - c*psi_dot
    Driven by the applied voltage and the measured gyro rate only."""
    _, _, (E, F, H) = plant()
    c = E[0, 1] / E[0, 0]
    return dict(c=c, a=F[0, 0] / E[0, 0], b=H[0, 0] / E[0, 0],
                g=(F[0, 0] * c - F[0, 1]) / E[0, 0])


def loop_margins(K, delay_ticks=1.0):
    """Loop gain broken at the controller output (nominal plant, delay,
    observer).  Returns (crossover Hz, phase margin deg, |L| at 10 Hz)."""
    A, B, _ = plant()
    Ad, Bd = c2d(A, B)
    o = observer_coeffs()
    fs = np.linspace(0.2, 50.0, 2000)
    L = []
    for f in fs:
        z = np.exp(1j * 2 * np.pi * f * TS)
        P = np.linalg.solve(z * np.eye(4) - Ad, Bd).flatten() * z ** (-delay_ticks)
        psi, psid = P[1], P[3]
        zo = TS * (o['b'] / z + o['g'] * psid) / (z - (1 - TS * o['a']))
        thd = zo - o['c'] * psid
        th = TS * thd / (z - 1)
        L.append(K[0] * th + K[1] * psi + K[2] * thd + K[3] * psid + K[4] / z)
    L = np.array(L)
    mag, ph = np.abs(L), np.degrees(np.angle(L))
    cross = [(fs[i], 180.0 + ((ph[i] + 180.0) % 360.0 - 180.0))
             for i in range(1, len(fs)) if (mag[i - 1] - 1) * (mag[i] - 1) < 0]
    fc, pm = cross[-1] if cross else (float('nan'), float('nan'))
    return fc, pm, mag[np.argmin(np.abs(fs - 10.0))]


def closed_loop_rho(K, true, vb=1.0):
    """Spectral radius of true plant + delay + nominal observer + controller
    (the true wheel-angle mode is output-only and excluded)."""
    A, B, _ = plant(**true)
    Ad, Bd = c2d(A, B)
    o = observer_coeffs()
    k = K
    Acl = np.zeros((7, 7))           # x(4), u_prev, z, theta_hat
    Acl[:4, :4], Acl[:4, 4:5] = Ad, Bd * vb
    Acl[4, 1] = -k[1]
    Acl[4, 3] = -k[3] + k[2] * o['c']
    Acl[4, 4] = -k[4]
    Acl[4, 5] = -k[2]
    Acl[4, 6] = -k[0]
    Acl[5, 5] = 1 - TS * o['a']
    Acl[5, 4] = TS * o['b']
    Acl[5, 3] = TS * o['g']
    Acl[6, 6], Acl[6, 5], Acl[6, 3] = 1.0, TS, -TS * o['c']
    ev = np.linalg.eigvals(Acl)
    ev = ev[np.abs(ev - 1.0) > 1e-9]
    return np.abs(ev).max()


def simulate(K, true, tilt0_deg=5.0, T=8.0, vb=1.0, dead=0.0, comp=0.0,
             slew=1.5, tau=0.8, push=0.0, h=0.2475):
    """Nonlinear plant (RK4), complementary filter with the accelerometer
    corrupted by base + tangential acceleration at sensor height h, motor
    static-friction deadband, smooth deadband compensation, slew + clamp."""
    M, m, L, Jr = true['M'], true['m'], true['L'], true['Jr']
    R, g = R_WHEEL, 9.81
    Jpsi, Jw = M * L * L / 3.0, m * R * R / 2.0
    alpha, beta = KT / R_TOTAL, KT * KE / R_TOTAL

    def f(x, v, fext):
        _, psi, thd, psid = x
        E = np.array([[(2 * m + M) * R * R + 2 * Jw + 2 * Jr, M * L * R * np.cos(psi) - 2 * Jr],
                      [M * L * R * np.cos(psi) - 2 * Jr, M * L * L + Jpsi + 2 * Jr]])
        rhs = [2 * alpha * v - 2 * beta * (thd - psid) + M * L * R * psid ** 2 * np.sin(psi),
               -2 * alpha * v + 2 * beta * (thd - psid) + M * g * L * np.sin(psi) + fext * L]
        acc = np.linalg.solve(E, rhs)
        return np.array([thd, psid, acc[0], acc[1]])

    o = observer_coeffs()
    rng = np.random.default_rng(1)
    x = np.array([0.0, np.radians(tilt0_deg), 0.0, 0.0])
    z = th_hat = u_prev = 0.0
    psi_hat, xdot = x[1], np.zeros(4)
    a_cf = tau / (tau + TS)
    log = []
    for i in range(int(T / TS)):
        gyro = x[3] + rng.standard_normal() * np.radians(0.3)
        psi_acc = x[1] - (xdot[2] * R + h * xdot[3]) / g + rng.standard_normal() * np.radians(0.5)
        psi_hat = a_cf * (psi_hat + gyro * TS) + (1 - a_cf) * psi_acc
        thd_hat = z - o['c'] * gyro
        u = -(K[0] * th_hat + K[1] * psi_hat + K[2] * thd_hat + K[3] * gyro + K[4] * u_prev)
        u = np.clip(np.clip(u, -V_NOMINAL, V_NOMINAL), u_prev - slew, u_prev + slew)
        z += TS * (o['b'] * u_prev - o['a'] * z + o['g'] * gyro)
        th_hat += TS * thd_hat
        v = (u_prev + comp * np.clip(u_prev / 0.3, -1, 1)) * vb
        v = 0.0 if abs(v) < dead else v - np.sign(v) * dead
        fext = push if 3.0 <= i * TS < 3.1 else 0.0
        hh = TS / 5
        for _ in range(5):
            k1 = f(x, v, fext)
            k2 = f(x + hh / 2 * k1, v, fext)
            k3 = f(x + hh / 2 * k2, v, fext)
            k4 = f(x + hh * k3, v, fext)
            x = x + hh / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        xdot = f(x, v, fext)
        u_prev = u
        log.append((np.degrees(x[1]), x[0] * R, u))
        if abs(x[1]) > np.radians(45):
            return np.array(log), False
    return np.array(log), True


def main():
    K = design()
    o = observer_coeffs()
    d = np.pi / 180.0
    A, _, _ = plant()
    print(f"motor: Ke={KE:.3f} V/(rad/s)  Kt={KT:.3f} N m/A  R={R_MOTOR:.2f} ohm + driver {R_DRIVER:.2f} ohm")
    print(f"open-loop poles (1/s): {np.round(np.sort(np.linalg.eigvals(A).real), 2)}")
    print("\n// ---- paste into BigWheelBalance.ino (generated by design_gains.py) ----")
    print(f"static const float kK_tilt = {-K[1] * d:.4f}f;        // V per deg of tilt")
    print(f"static const float kK_tiltRate = {-K[3] * d:.5f}f;   // V per deg/s")
    print(f"static const float kK_wheelPos = {-K[0] / R_WHEEL:.4f}f;     // V per m of travel")
    print(f"static const float kK_wheelVel = {-K[2] / R_WHEEL:.4f}f;    // V per m/s")
    print(f"static const float kK_prevCmd = {-K[4]:.4f}f;     // per V of last command")
    print(f"static const float kObsCouple = {o['c']:.5f}f;")
    print(f"static const float kObsDecay = {o['a']:.4f}f;")
    print(f"static const float kObsInputGain = {o['b']:.4f}f;")
    print(f"static const float kObsRateGain = {o['g']:.4f}f;")
    print(f"static const float kDriveFeedforwardVoltsPerMps = {KE / R_WHEEL:.4f}f;  // back-EMF at speed")
    print("// ----------------------------------------------------------------------\n")

    fc, pm, l10 = loop_margins(K)
    print(f"loop: crossover {fc:.1f} Hz, phase margin {pm:.0f} deg, |L| at 10 Hz {l10:.2f}"
          "  (keep crossover ~4-6 Hz, |L|@10Hz < 0.6)")
    nom = dict(M=M_BODY, m=M_WHEEL, L=L_COM, Jr=J_ROTOR)
    print(f"nominal closed-loop spectral radius: {closed_loop_rho(K, nom):.4f} (<1 = stable)")
    print("robustness (ok = stable) across body mass x CoM height, three motor/battery corners:")
    Ms = [M_BODY * s for s in (0.6, 0.8, 1.0, 1.25, 1.5)]
    Ls = [L_COM * s for s in (0.6, 0.75, 1.0, 1.3, 1.6)]
    for Jr, vb, label in [(J_ROTOR * 0.4, 1.15, "low rotor inertia, full pack"),
                          (J_ROTOR, 1.0, "nominal"),
                          (J_ROTOR * 2.4, 0.85, "high rotor inertia, low pack")]:
        print(f"  [{label}]  L(m) -> " + " ".join(f"{L:5.3f}" for L in Ls))
        for M in Ms:
            row = [closed_loop_rho(K, dict(M=M, m=M_WHEEL, L=L, Jr=Jr), vb) for L in Ls]
            print(f"    M={M:4.2f} kg          " +
                  " ".join("  ok " if r < 0.9995 else " BAD " for r in row))

    if "--sim" in sys.argv:
        print("\nnonlinear sims (tilt in deg, travel in cm):")
        cases = [("5 deg start", {}), ("10 deg start", dict(tilt0_deg=10)),
                 ("pack +15%", dict(vb=1.15)), ("pack -15%", dict(vb=0.85)),
                 ("1 V deadband", dict(dead=1.0)),
                 ("1 V deadband, 1 V comp", dict(dead=1.0, comp=1.0)),
                 ("12 N push", dict(push=12.0))]
        for name, kw in cases:
            lg, ok = simulate(K, nom, **kw)
            tail = lg[-400:]
            print(f"  {name:24s} {'held' if ok else 'FELL'}  peak V {np.abs(lg[:, 2]).max():5.2f}"
                  f"  max travel {np.abs(lg[:, 1]).max() * 100:5.1f}"
                  f"  last-2s tilt pk {np.abs(tail[:, 0]).max():4.2f}")


if __name__ == "__main__":
    main()
