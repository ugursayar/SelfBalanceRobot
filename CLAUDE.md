# SelfBalanceRobot

Arduino firmware for a MakeBlock MegaPi (ATmega2560) two-wheel self-balancing
robot. Gyro (MPU6050 `MeGyro`) on RJ25 `PORT_6`, right motor/encoder on slot 1,
left on slot 2.

## Two firmware targets

| Folder | What it is | When to touch it |
|---|---|---|
| `SelfBalanceRobot/` | Full firmware: small testable modules wired together by `SelfBalanceRobot.ino`. USB + Bluetooth command channel, EEPROM balance point, auto-arm, telemetry, runtime stats, selectable PID **or** LQR law. | Structural / feature work; anything that needs unit tests or serial diagnostics. |
| `LqrBalance/` | Standalone single-file pure-LQR sketch (`u = K·x`, 4 states, encoders live). No serial, Bluetooth, or telemetry by design. | On-robot balance tuning. This is the sketch that achieved stable, calm balance. |

`LqrBalance/CLAUDE.md` holds the hard-won hardware traps and tuning decisions
for the LQR sketch (PWM timer setup, coast vs brake, wheel-gain signs, battery
voltage as hidden gain, slew limiter, watchdog ordering). Read it before
changing that sketch. `LqrBalance/theory.md` derives `K` from the plant model.

## Current state (code is the source of truth, docs lag)

- `SelfBalanceRobot/config.h` currently has `BareBalanceFirmware = true` and
  `EnableLqrController = true`. `README.md` and `docs/bring-up.md` still describe
  the diagnostic profile (`BareBalanceFirmware = false`, PID) as the default.
- In the bare profile, Bluetooth control and balance-point learning are off
  (`!BareBalanceFirmware`); USB `Serial` commands at 115200 still work.
- Because of that profile flip, `test_config` and `test_balance_pipeline` fail
  (they pin the diagnostic/PID profile). The other 16 native tests pass.

## Build and flash

arduino-cli is installed; its data dir is `D:\packages\arduino` and the
Makeblock library (`MeMegaPi.h`) is in `user\libraries\MakeBlockDrive`. A second
copy, `MakeBlock_Drive_Updated`, defines the same classes; be aware which one
resolves if behavior changes.

```
arduino-cli board list                                    # find the port (has been COM8; it moves)
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 LqrBalance
arduino-cli upload  -p <PORT> --fqbn arduino:avr:mega:cpu=atmega2560 LqrBalance
```

Same commands with `SelfBalanceRobot` for the full firmware. Run from the repo root.

## Native unit tests

Host-side tests for the `SelfBalanceRobot/` modules live in `tests/native/`
(plain `assert`, one `test_<module>.cpp` per module, `-Werror`). The Makefile
generates stub `Arduino.h` / `MeGyro.h` into `build/`.

```
cd tests/native && make            # all tests
make test_lqr_controller           # one test
make clean
```

On this Windows box the MSYS g++ needs a writable temp dir or it fails with
`Cannot create temporary file in C:\WINDOWS\`. Exporting `TMP` in Git Bash is
not picked up; set it from PowerShell instead:

```powershell
$env:TMP = $env:TEMP = "<writable dir>"; make -C tests/native
```

When adding a module to `SelfBalanceRobot/`, keep hardware calls in the `.ino`
(or a thin hardware wrapper), put the logic in a pure class, and add a
`test_<module>.cpp` plus a Makefile target (and list it in `.PHONY` and `all`).
`LqrBalance/` has no native tests; it is only syntax-checked.

## Conventions

- All tunables are named constants: `constexpr` in the `Config` namespace
  (`config.h`) for the full firmware, `static const k...` at the top of
  `LqrBalance.ino`. Don't scatter magic numbers.
- Strict `float`: every literal gets an `f` suffix; cast library `double`s to
  `float` (AVR `double` is software-emulated).
- 200 Hz control loop (5 ms). Keep the hot loop free of serial printing; the
  full firmware uses on-demand `STATUS` snapshots instead of streaming.
- Motor/encoder signs are wiring-specific (`right = +u`, `left = -u`). Verify on
  hardware before changing either.

## Working on the physical robot

- The user tunes live: they name a value, you edit, compile, upload, then they
  test. Change one constant at a time.
- After every on-robot test, ask what they physically saw (held/fell, wobble,
  direction, whether a hand or the USB cable touched it) before trusting any
  telemetry. The cable and hands have repeatedly corrupted captures.
- Battery charge changes the effective loop gain (torque per PWM scales with
  pack voltage). A config tuned on a drained pack is over-driven when full, so
  note the charge level when comparing runs.
- Hold the robot at its balance point through power-up/reset: the gyro bias
  and (in `LqrBalance`) the balance reference are latched then.

## Docs

- `docs/bring-up.md` — hardware bring-up, serial command reference, release technique
- `docs/specs/` — design specs; `docs/plans/` — implementation plans (historical)
- `docs/logs/` — raw serial captures from the June 2026 PID tuning sessions
