/*
 * BigWheelBalance — balance firmware for the big-wheel build of the MakeBlock
 * MegaPi (ATmega2560) robot: Makeblock 36 mm 12 V DC gear motors on the MegaPi
 * DC terminals (NO encoders), 12.5 cm wheels, MPU6050 31 cm above the floor.
 * Serial-free, like LqrBalance: power on held at the balance point, and it
 * balances.  The LED blinks faster as the tilt nears the balance point, goes
 * solid when armed, and resumes blinking after a fall.
 *
 * ---------------------------------------------------------------------------
 * Control law: discrete LQR with a model-based wheel observer.
 *
 *     u[V] = kK_tilt*tiltErr + kK_tiltRate*tiltRate
 *          + kK_wheelPos*travelHat + kK_wheelVel*speedHat + kK_prevCmd*uPrev
 *
 * There are no encoders, so travel and speed are ESTIMATED from the voltage we
 * apply and the gyro rate, using the wheel row of the voltage-driven pendulum
 * model (the motor's back-EMF couples wheel speed into the dynamics):
 *
 *     z'       = kObsInputGain*u - kObsDecay*z + kObsRateGain*tiltRate
 *     wheelHat = z - kObsCouple*tiltRate        (wheel angular speed, rad/s)
 *
 * A slow balance-point trim then slides the tilt setpoint until the position
 * term is no longer needed, so a hand-held arming error does not leave the
 * robot parked away from (or driving off from) its arm point.
 *
 * Gains and observer constants come from design_gains.py (model + LQR at
 * 200 Hz with one tick of delay -> the kK_prevCmd term).  The command is
 * computed in VOLTS and converted to PWM with kBatteryVolts, so the gains are
 * physical and a battery change is one constant.  Signs: positive u drives the
 * robot "forward" and positive tilt is a forward lean; all K are positive
 * except kK_prevCmd.  See theory.md for the derivation and README.md for
 * bring-up and tuning.
 *
 * The IMU is read directly (not via MeGyro) so we control the MPU6050 low-pass,
 * the I2C clock and timeout, and the complementary-filter time constant.
 */

#include <MeMegaPi.h>  // also declares Wire (Makeblock's old utility/Wire.h)
#include <avr/io.h>
#include <avr/wdt.h>

// Makeblock's bundled Wire.h shadows the core one and lacks setWireTimeout(),
// but the linked implementation is the core's (AVR 1.8.8), so call its C entry
// point directly.  Without a timeout a stuck bus blocks until the watchdog
// reboots the board; with it, a glitch just fails that read.
extern "C" void twi_setTimeoutInMicros(uint32_t timeout, bool resetWithTimeout);

// Break any watchdog-reset loop the instant the chip boots, before setup()
// re-arms it (a watchdog reset leaves the WDT enabled).
void disableWatchdogOnBoot()
    __attribute__((naked)) __attribute__((section(".init3")));
void disableWatchdogOnBoot() {
  MCUSR = 0;
  wdt_disable();
}

// ===========================================================================
// Tuning constants  (all the knobs live here)
// ===========================================================================

// --- Hardware map (confirmed with BigWheelBringUp, 2026-10-01) ---------------
// MegaPi DC terminals are PORT1A..PORT4A (the library has no "5A/6A").
static const uint8_t kRightMotorPort = PORT2A;
static const uint8_t kLeftMotorPort = PORT1A;
// +1/-1 so that a POSITIVE command rolls each wheel the way that moves the
// robot "forward".  Both wheels must agree.  Bench: +PWM rolls the right wheel
// backward and the left wheel forward.
static const int8_t kRightMotorSign = -1;
static const int8_t kLeftMotorSign = 1;

// --- IMU (MPU6050) ------------------------------------------------------------
static const uint8_t kImuAddress = 0x68;
static const uint32_t kI2cClockHz = 400000UL;  // drop to 100000UL if reads fail
static const uint16_t kI2cTimeoutMicros = 3000;
// Axis map (0 = X, 1 = Y, 2 = Z), set so that leaning FORWARD makes both the
// tilt and the tilt rate positive.  Bench (BigWheelBringUp): the robot pitches
// about gyro Y; a forward lean drives accel X negative and gyro Y positive (the
// opposite of MeGyro's angleX convention).  The chip answers WHO_AM_I 0x98, an
// MPU6050-compatible part; the register map used here works on it.
static const uint8_t kAccelFwdAxis = 0;
static const float kAccelFwdSign = -1.0f;
static const uint8_t kGyroPitchAxis = 1;
static const float kGyroPitchSign = 1.0f;
// MPU6050 digital low-pass: 2 = 94/98 Hz (~3 ms delay).  1 is noisier, 3 adds
// ~5 ms of delay; the gains assume about one 5 ms tick of total delay.
static const uint8_t kImuDlpfConfig = 2;
static const float kAccelLsbPerG = 8192.0f;      // +-4 g range
static const float kGyroLsbPerDps = 65.5f;       // +-500 deg/s range
// Complementary filter time constant.  The sensor is 25 cm above the axle, so
// the accelerometer reads wheel and body acceleration as tilt; a long,
// gyro-dominant time constant averages that out.
static const float kTiltFilterTauSec = 0.8f;
// Ignore the accelerometer for a tick when |a| is this far from 1 g.
static const float kAccelGateG = 0.25f;
static const float kRateClampDegPerSec = 250.0f;
// Boot calibration only counts while the robot is held still: over the ~1 s
// window the pitch/yaw rates and the accel tilt must each stay within these
// peak-to-peak spreads, else it retries (LED toggles once per attempt).  A reset
// while balancing once calibrated mid-fall: a 27 deg/s false bias drove the
// command to the rail.
static const float kCalMaxRateSpreadDps = 10.0f;
static const float kCalMaxTiltSpreadDeg = 3.0f;

// --- Loop timing ----------------------------------------------------------------
static const uint32_t kLoopMicros = 5000UL;  // 200 Hz (the gains are designed for it)

// --- LQR gain row + wheel observer (generated by design_gains.py) ------------
// Estimated plant: body 2.0 kg, CoM 0.14 m above the axle, wheel r 0.0625 m,
// 36 mm 12 V 240 rpm motors.  Re-run the script with measured values.
// Weights tilt 5 deg / rate 150 deg/s -> loop crossover ~5.6 Hz.  The first
// set (crossover 11 Hz) held a 10 Hz limit-cycle shiver near upright.
static const float kK_tilt = 1.4686f;        // V per deg of tilt
static const float kK_tiltRate = 0.14361f;   // V per deg/s
static const float kK_wheelPos = 10.8816f;    // V per m of (estimated) travel
static const float kK_wheelVel = 29.7342f;   // V per m/s of (estimated) speed
static const float kK_prevCmd = -0.1827f;    // per V of last command (delay comp)
static const float kObsCouple = 0.87912f;
static const float kObsDecay = 9.0782f;
static const float kObsInputGain = 19.0133f;
static const float kObsRateGain = 17.0590f;
static const float kWheelRadiusM = 0.0625f;

// The travel estimate integrates a model, so it can drift.  Clamp how hard the
// position term may lean the robot back, and optionally let old displacement
// leak away (0 = hold the arm point; e.g. 0.2 = forget over ~5 s).
static const float kWheelPosClampVolts = 4.0f;
static const float kWheelPosLeakPerSec = 0.0f;

// Balance-point trim.  The setpoint captured at arm is wherever your hand held
// it; an error of d degrees makes the position term hold the robot ~21 cm/deg
// away from the arm point (and past ~2 deg the clamp above saturates and it
// drives away).  The trim slides the setpoint until the position term is no
// longer needed, i.e. onto the true balance point, with this time constant
// (0 = off).  Simulated: a 3 deg arming error returns to the arm point.
static const float kBalanceTrimTauSec = 4.0f;
static const float kMaxTrimDeg = 6.0f;        // max trim away from the arm tilt

// --- Output stage ------------------------------------------------------------
// Pack voltage the PWM is scaled to (PWM 255 = this many volts).  Set it to the
// actual pack voltage to keep the loop gain constant across charge levels.
static const float kBatteryVolts = 12.0f;
static const float kMaxVolts = 12.0f;             // command clamp
// Slew limit per 5 ms tick.  Abrupt steps excited backlash on the old build;
// 1.5 V/tick ~ 32 PWM/tick, close to the proven 30.
static const float kMaxVoltStepPerTick = 1.5f;
// Static-friction (deadband) compensation, added smoothly: the full
// kDeadbandCompVolts once |u| exceeds kDeadbandRampVolts, linear below that, so
// the command passes through zero without a step (a hard minimum-PWM floor
// caused reverse kicks on the old build).  Raise it if the robot holds a small
// steady wobble near upright.
static const float kDeadbandCompVolts = 0.0f;    // B: 0 (C = 0.15 V halved the 2 Hz sway; 0.3 V on the 11 Hz loop shivered)
static const float kDeadbandRampVolts = 0.3f;

// --- Bluetooth drive -----------------------------------------------------------
// Makeblock Bluetooth controller through the Me Bluetooth module on Serial3,
// MePS2 packet format (FF 55, LX, btn, LY, btn, RX, btn, RY, checksum; ~80
// packets/s; sticks 0..255, centre 128, up/left = 0) -- bench-verified with
// BigWheelBringUp -DBRINGUP_BT.  false = balance only (the known-good build).
static const bool kEnableBluetoothDrive = true;
static const uint32_t kBluetoothBaud = 115200UL;
static const uint32_t kDriveTimeoutMs = 300UL;  // no valid packet -> sticks centred
static const uint8_t kStickDeadband = 10;       // counts around the 128 centre
// Left stick Y = speed.  The speed reference ramps at kDriveAccelMps2 and the
// travel reference integrates it, so the LQR leans into the motion and holds
// the spot where the stick is released.  At speed the motor needs its back-EMF
// voltage: feedforward Ke / r (design_gains.py).
static const float kMaxDriveSpeedMps = 0.8f;  // sim: 7.1 V peak, ~5 V braking headroom; 1.2 m/s nears the 11 V stall cutoff
static const float kDriveAccelMps2 = 0.5f;
static const float kDriveFeedforwardVoltsPerMps = 7.6394f;
// Right stick X = turn rate (left = turn left).  Yaw loop on the gyro:
// differential volts = feedforward (no-load, ~0.22 m track) + P on the rate
// error + P on the heading error (the integral of the rate error), so the
// robot holds its heading, returns after being twisted, and drives straight;
// the stick rotates the heading it holds.
static const float kMaxTurnRateDps = 90.0f;
static const float kTurnAccelDps2 = 360.0f;
static const uint8_t kGyroYawAxis = 2;
static const float kGyroYawSign = 1.0f;  // + = turning left (CCW from above), bench-verified
static const float kTurnFeedforwardVoltsPerDps = 0.0147f;
static const float kTurnP = 0.02f;               // V per deg/s of yaw-rate error
// Heading hold.  Yaw plant estimate: ~68 deg/s per V, ~70 ms lag (back-EMF
// damping).  0.10 V/deg returns a twist ~63% in ~0.6 s with no overshoot,
// robust to 0.5-2x turn gain and 0.03-0.2 s lag (simulated).  The heading
// memory is capped so a long, hard twist cannot wind up a violent snap-back.
static const float kHeadingP = 0.10f;            // V per deg of heading error
static const float kMaxHeadingErrDeg = 30.0f;
static const float kMaxTurnVolts = 3.0f;

// --- Arm / safety ------------------------------------------------------------
static const float kBalancePointDeg = 0.0f;   // fallback if boot tilt is unusable
static const float kMaxBootTiltDeg = 15.0f;   // boot tilt beyond this -> fallback
static const float kArmWindowDeg = 4.0f;      // engage when within this of center
static const float kArmMaxRateDegPerSec = 30.0f;
static const float kFallAngleDeg = 35.0f;     // disengage past this
static const uint8_t kMaxImuFailTicks = 3;    // consecutive bad reads -> motors off
// Stall protection.  The 36 mm motors stall near 5 A and the MegaPi driver
// modules are rated ~1 A continuous / 2 A peak.  A command pinned at the rail
// this long means the robot is stuck (or the motors are not responding): cut
// the motors, and do not re-arm until the robot has tipped past the fall angle
// (picked up / laid down), so a stuck robot cannot keep hammering the drivers.
// Normal hard-push recoveries saturate for well under 0.5 s.
static const float kSaturationVolts = 11.0f;
static const float kSaturationTripSec = 1.5f;

// --- Arming LED ----------------------------------------------------------------
static const float kLedFarDeg = 20.0f;
static const uint32_t kLedSlowMs = 500UL;
static const uint32_t kLedFastMs = 70UL;
static const uint32_t kLedErrorMs = 40UL;     // IMU missing at boot

// ===========================================================================
// Globals
// ===========================================================================
static const float kDegToRad = 0.017453293f;
static const float kPwmPerVolt = 255.0f / kBatteryVolts;

static MeMegaPiDCMotor gRight;
static MeMegaPiDCMotor gLeft;

static bool gImuOk = false;
static float gGyroBiasDps = 0.0f;
static float gYawBiasDps = 0.0f;
static float gTiltDeg = 0.0f;               // complementary-filter tilt
static float gBalancePointDeg = kBalancePointDeg;
static float gSetpointDeg = kBalancePointDeg;  // trimmed while armed
static float gArmTiltDeg = kBalancePointDeg;   // tilt captured at arm
static bool gArmed = false;
static uint8_t gImuFailTicks = 0;
static float gSaturatedSec = 0.0f;          // time the command has been pinned
static bool gSaturationLockout = false;     // set by a stall trip
static uint32_t gLastTickMicros = 0;

// Observer + output state (reset on every arm).
static float gObsZ = 0.0f;                  // wheel-speed observer state (rad/s)
static float gWheelAngleHat = 0.0f;         // estimated wheel angle (rad)
static float gAppliedVolts = 0.0f;          // linear command applied last tick
static float gPrevFeedforward = 0.0f;       // drive feedforward inside gAppliedVolts
static int16_t gLastPwmRight = INT16_MIN;   // forces the first write
static int16_t gLastPwmLeft = INT16_MIN;

// Drive references (reset on every arm) and controller input.
static float gSpeedRef = 0.0f;              // m/s
static float gTravelRef = 0.0f;             // m
static float gTurnRef = 0.0f;               // deg/s, + = left
static float gHeadingErrDeg = 0.0f;         // integral of turnRef - yawRate
static uint8_t gStickLY = 128;
static uint8_t gStickRX = 128;
static uint32_t gLastPacketMs = 0;
static uint8_t gPkt[8];
static uint8_t gPktIndex = 0;
static uint8_t gPktState = 0;               // 0 want FF, 1 want 55, 2 payload

static bool gLedOn = false;
static uint32_t gLedToggleMs = 0;

// ===========================================================================
// Helpers
// ===========================================================================
static inline float fabsFast(float v) { return v < 0.0f ? -v : v; }

static inline float clampF(float v, float limit) {
  if (v > limit) return limit;
  if (v < -limit) return -limit;
  return v;
}

static void writeImuReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kImuAddress);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Burst-read accel (0..2), temperature (3) and gyro (4..6) raw counts.
static bool readImu(int16_t raw[7]) {
  Wire.beginTransmission(kImuAddress);
  Wire.write(static_cast<uint8_t>(0x3B));
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(kImuAddress, static_cast<uint8_t>(14)) != 14) {
    return false;
  }
  for (uint8_t i = 0; i < 7; ++i) {
    const uint8_t hi = static_cast<uint8_t>(Wire.read());  // two statements:
    const uint8_t lo = static_cast<uint8_t>(Wire.read());  // read order matters
    raw[i] = static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
  }
  return true;
}

// Accelerometer-only tilt (deg).  Sign-agnostic to the up axis, like MeGyro.
static float accelTiltDeg(const int16_t raw[7], float* accelMagSqG2) {
  const float af = kAccelFwdSign * static_cast<float>(raw[kAccelFwdAxis]);
  float sideSq = 0.0f;
  for (uint8_t i = 0; i < 3; ++i) {
    if (i != kAccelFwdAxis) {
      const float a = static_cast<float>(raw[i]);
      sideSq += a * a;
    }
  }
  const float invLsbSq = 1.0f / (kAccelLsbPerG * kAccelLsbPerG);
  *accelMagSqG2 = (af * af + sideSq) * invLsbSq;
  return static_cast<float>(atan2(af, sqrt(sideSq))) * (1.0f / kDegToRad);
}

static inline float gyroRateDps(const int16_t raw[7]) {
  return kGyroPitchSign * static_cast<float>(raw[4 + kGyroPitchAxis]) *
         (1.0f / kGyroLsbPerDps);
}

static inline float yawRateRawDps(const int16_t raw[7]) {
  return kGyroYawSign * static_cast<float>(raw[4 + kGyroYawAxis]) *
         (1.0f / kGyroLsbPerDps);
}

// Non-blocking MePS2-format parser; keeps the latest valid left-Y / right-X.
static void pollController() {
  while (Serial3.available() > 0) {
    const uint8_t c = static_cast<uint8_t>(Serial3.read());
    if (gPktState == 2) {
      gPkt[gPktIndex++] = c;
      if (gPktIndex == 8) {
        uint8_t sum = 0;
        for (uint8_t i = 0; i < 7; ++i) {
          sum = static_cast<uint8_t>(sum + gPkt[i]);
        }
        if (sum == gPkt[7]) {
          gStickLY = gPkt[2];
          gStickRX = gPkt[4];
          gLastPacketMs = millis();
        }
        gPktState = 0;
      }
    } else if (c == 0xFF) {
      gPktState = 1;
    } else if (gPktState == 1 && c == 0x55) {
      gPktState = 2;
      gPktIndex = 0;
    } else {
      gPktState = 0;
    }
  }
}

// Stick byte -> -1..+1 (0 -> -1, 255 -> +1) with a centre deadband.
static float stickToUnit(uint8_t value) {
  const int16_t d = static_cast<int16_t>(value) - 128;
  if (d <= kStickDeadband && d >= -static_cast<int16_t>(kStickDeadband)) {
    return 0.0f;
  }
  const float span = 127.0f - static_cast<float>(kStickDeadband);
  const float mag = static_cast<float>(d > 0 ? d - kStickDeadband : d + kStickDeadband);
  return clampF(mag / span, 1.0f);
}

static inline float rampToward(float current, float target, float maxStep) {
  return current + clampF(target - current, maxStep);
}

// Bring up the MPU6050, measure gyro bias and the boot tilt (robot held still
// at its balance point).  Returns false if the sensor does not answer.
static bool initImu() {
  Wire.begin();
  Wire.setClock(kI2cClockHz);
  twi_setTimeoutInMicros(kI2cTimeoutMicros, true);
  writeImuReg(0x6B, 0x01);            // wake, clock = PLL on gyro X
  delay(50);
  writeImuReg(0x19, 0x00);            // sample-rate divider: 1 kHz
  writeImuReg(0x1A, kImuDlpfConfig);  // digital low-pass
  writeImuReg(0x1B, 0x08);            // gyro +-500 deg/s
  writeImuReg(0x1C, 0x08);            // accel +-4 g
  delay(50);

  // Retry until a whole window is still; give up only if the sensor is silent.
  for (;;) {
    int16_t raw[7];
    const uint16_t kSamples = 200;    // ~1 s
    float biasSum = 0.0f;
    float yawBiasSum = 0.0f;
    float tiltSum = 0.0f;
    float rateMin = 1e9f, rateMax = -1e9f;
    float yawMin = 1e9f, yawMax = -1e9f;
    float tiltMin = 1e9f, tiltMax = -1e9f;
    uint16_t good = 0;
    for (uint16_t i = 0; i < kSamples; ++i) {
      if (readImu(raw)) {
        float magSq;
        const float rate = gyroRateDps(raw);
        const float yaw = yawRateRawDps(raw);
        const float tilt = accelTiltDeg(raw, &magSq);
        biasSum += rate;
        yawBiasSum += yaw;
        tiltSum += tilt;
        if (rate < rateMin) rateMin = rate;
        if (rate > rateMax) rateMax = rate;
        if (yaw < yawMin) yawMin = yaw;
        if (yaw > yawMax) yawMax = yaw;
        if (tilt < tiltMin) tiltMin = tilt;
        if (tilt > tiltMax) tiltMax = tilt;
        ++good;
      }
      delay(5);
    }
    if (good < kSamples / 2) {
      return false;
    }
    if ((rateMax - rateMin) <= kCalMaxRateSpreadDps &&
        (yawMax - yawMin) <= kCalMaxRateSpreadDps &&
        (tiltMax - tiltMin) <= kCalMaxTiltSpreadDeg) {
      gGyroBiasDps = biasSum / static_cast<float>(good);
      gYawBiasDps = yawBiasSum / static_cast<float>(good);
      gTiltDeg = tiltSum / static_cast<float>(good);
      return true;
    }
    gLedOn = !gLedOn;  // moved: show an attempt went by, then try again
    digitalWrite(LED_BUILTIN, gLedOn ? HIGH : LOW);
  }
}

// MeMegaPiDCMotor sets its PWM timers in the constructor, which runs before the
// Arduino core's init() overwrites them, so set them again here: 8-bit fast
// PWM, /64 = 976 Hz (the library's frequency) on Timers 1-4.  Timer 0 (pin 4,
// PORT4A) already runs at 976 Hz and also drives millis(); leave it alone.
static void configureDcMotorPwmTimers() {
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS22);
  TCCR3A = _BV(WGM30);
  TCCR3B = _BV(CS31) | _BV(CS30) | _BV(WGM32);
  TCCR4A = _BV(WGM40);
  TCCR4B = _BV(CS41) | _BV(CS40) | _BV(WGM42);
}

// Volts -> PWM for one wheel: smooth deadband compensation, round, clamp.
static int16_t voltsToPwm(float volts) {
  float out = volts;
  if (kDeadbandCompVolts > 0.0f) {
    out += kDeadbandCompVolts * clampF(volts / kDeadbandRampVolts, 1.0f);
  }
  float pwm = out * kPwmPerVolt;
  pwm += pwm >= 0.0f ? 0.5f : -0.5f;  // round to nearest
  return static_cast<int16_t>(clampF(pwm, 255.0f));
}

// Drive each wheel ("forward" PWM); skip identical re-writes.
static inline void driveWheels(int16_t pwmRight, int16_t pwmLeft) {
  if (pwmRight != gLastPwmRight) {
    gLastPwmRight = pwmRight;
    gRight.run(static_cast<int16_t>(kRightMotorSign * pwmRight));
  }
  if (pwmLeft != gLastPwmLeft) {
    gLastPwmLeft = pwmLeft;
    gLeft.run(static_cast<int16_t>(kLeftMotorSign * pwmLeft));
  }
}

static inline void driveMotors(int16_t pwm) { driveWheels(pwm, pwm); }

static inline void resetBalanceState() {
  gObsZ = 0.0f;
  gWheelAngleHat = 0.0f;
  gAppliedVolts = 0.0f;
  gPrevFeedforward = 0.0f;
  gSpeedRef = 0.0f;
  gTravelRef = 0.0f;
  gTurnRef = 0.0f;
  gHeadingErrDeg = 0.0f;
  gSaturatedSec = 0.0f;
}

// Solid while armed; while disarmed, blink faster the closer the tilt is to the
// balance point (arming aid).
static void updateArmingLed(float offsetDeg, bool armed) {
  const uint32_t now = millis();
  if (armed) {
    if (!gLedOn) {
      gLedOn = true;
      digitalWrite(LED_BUILTIN, HIGH);
    }
    gLedToggleMs = now;
    return;
  }
  const float prox = fabsFast(offsetDeg);
  uint32_t period;
  if (prox <= kArmWindowDeg) {
    period = kLedFastMs;
  } else if (prox >= kLedFarDeg) {
    period = kLedSlowMs;
  } else {
    const float t = (prox - kArmWindowDeg) / (kLedFarDeg - kArmWindowDeg);
    period = kLedFastMs +
             static_cast<uint32_t>(t * static_cast<float>(kLedSlowMs - kLedFastMs));
  }
  if (now - gLedToggleMs >= period) {
    gLedToggleMs = now;
    gLedOn = !gLedOn;
    digitalWrite(LED_BUILTIN, gLedOn ? HIGH : LOW);
  }
}

// ===========================================================================
// Setup
// ===========================================================================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  gRight.reset(kRightMotorPort);
  gLeft.reset(kLeftMotorPort);
  configureDcMotorPwmTimers();
  driveMotors(0);

  // Hold the robot still at its balance point through this (~1.2 s).
  if (kEnableBluetoothDrive) {
    Serial3.begin(kBluetoothBaud);
  }
#ifdef BWB_DEBUG
  Serial.begin(115200);
  Serial.println(F("BWB_DEBUG 100Hz: tiltErr[cdeg] u[cV] rate[0.1dps] pwmR pwmL"));
#endif

  gImuOk = initImu();
  if (gImuOk && fabsFast(gTiltDeg) <= kMaxBootTiltDeg) {
    gBalancePointDeg = gTiltDeg;  // latch the boot pose as the balance point
  }
  gSetpointDeg = gBalancePointDeg;

  resetBalanceState();
  gLastTickMicros = micros();
  // Re-arm the watchdog only after the slow sensor bring-up; loop() pets it.
  wdt_enable(WDTO_250MS);
}

// ===========================================================================
// Control loop
// ===========================================================================
void loop() {
  wdt_reset();
  if (kEnableBluetoothDrive) {
    pollController();  // every pass, so the 64-byte RX buffer never overflows
  }

  if (!gImuOk) {  // sensor missing at boot: never drive, fast-blink forever
    driveMotors(0);
    const uint32_t now = millis();
    if (now - gLedToggleMs >= kLedErrorMs) {
      gLedToggleMs = now;
      gLedOn = !gLedOn;
      digitalWrite(LED_BUILTIN, gLedOn ? HIGH : LOW);
    }
    return;
  }

  const uint32_t nowMicros = micros();
  const uint32_t elapsedMicros = nowMicros - gLastTickMicros;
  if (elapsedMicros < kLoopMicros) {
    return;
  }
  gLastTickMicros = nowMicros;
  float dt = static_cast<float>(elapsedMicros) * 0.000001f;
  if (dt > 0.02f) {
    dt = 0.02f;  // a stall must not blow up the integrators
  }

  // ---- Tilt estimate ---------------------------------------------------------
  int16_t raw[7];
  if (!readImu(raw)) {
    if (++gImuFailTicks >= kMaxImuFailTicks) {
      gArmed = false;
      driveMotors(0);
    }
    return;  // a single glitch keeps the previous command for one tick
  }
  gImuFailTicks = 0;

  const float rateDps =
      clampF(gyroRateDps(raw) - gGyroBiasDps, kRateClampDegPerSec);
  float accelMagSqG2;
  const float accelTilt = accelTiltDeg(raw, &accelMagSqG2);
  const float gateLo = (1.0f - kAccelGateG) * (1.0f - kAccelGateG);
  const float gateHi = (1.0f + kAccelGateG) * (1.0f + kAccelGateG);
  gTiltDeg += rateDps * dt;
  if (accelMagSqG2 > gateLo && accelMagSqG2 < gateHi) {
    const float k = dt / (kTiltFilterTauSec + dt);
    gTiltDeg += k * (accelTilt - gTiltDeg);
  }

  updateArmingLed(gTiltDeg - gBalancePointDeg, gArmed);

  // ---- Arm gate: engage near the balance point while steady ------------------
  if (!gArmed) {
    driveMotors(0);
    if (gSaturationLockout) {  // after a stall trip: re-arm only once handled
      if (fabsFast(gTiltDeg - gBalancePointDeg) > kFallAngleDeg) {
        gSaturationLockout = false;
      }
      return;
    }
    if (fabsFast(gTiltDeg - gBalancePointDeg) <= kArmWindowDeg &&
        fabsFast(rateDps) <= kArmMaxRateDegPerSec) {
      gArmTiltDeg = gTiltDeg;
      gSetpointDeg = gTiltDeg;
      resetBalanceState();
      gArmed = true;
    }
    return;
  }

  const float tiltErr = gTiltDeg - gSetpointDeg;
  if (fabsFast(tiltErr) > kFallAngleDeg) {
    gArmed = false;
    driveMotors(0);
    return;
  }

  // ---- Wheel estimate (no encoders) -------------------------------------------
  const float rateRad = rateDps * kDegToRad;
  const float wheelRateHat = gObsZ - kObsCouple * rateRad;   // rad/s
  const float travelM = gWheelAngleHat * kWheelRadiusM;
  const float speedMps = wheelRateHat * kWheelRadiusM;

  // ---- Drive references from the controller -------------------------------------
  float speedTarget = 0.0f;
  float turnTarget = 0.0f;
  if (kEnableBluetoothDrive && (millis() - gLastPacketMs) < kDriveTimeoutMs) {
    speedTarget = -stickToUnit(gStickLY) * kMaxDriveSpeedMps;  // stick up = forward
    turnTarget = -stickToUnit(gStickRX) * kMaxTurnRateDps;     // stick left = turn left
  }
  gSpeedRef = rampToward(gSpeedRef, speedTarget, kDriveAccelMps2 * dt);
  gTurnRef = rampToward(gTurnRef, turnTarget, kTurnAccelDps2 * dt);
  // While driving, control speed only: the travel reference follows the robot,
  // so it never has to "catch up" to a reference it lagged during acceleration
  // (that catch-up overshot ~16% and rocked at full throttle).  Position hold
  // resumes at the spot where the speed ramp reaches zero.
  const bool driving = (speedTarget != 0.0f) || (fabsFast(gSpeedRef) > 0.001f);
  if (driving) {
    gTravelRef = travelM;
  }
  const float feedforward = kDriveFeedforwardVoltsPerMps * gSpeedRef;

  // ---- LQR state feedback (volts), about the moving drive reference ---------------
  const float posTerm =
      clampF(kK_wheelPos * (travelM - gTravelRef), kWheelPosClampVolts);
  float u = feedforward + (kK_tilt * tiltErr) + (kK_tiltRate * rateDps) +
            posTerm + (kK_wheelVel * (speedMps - gSpeedRef)) +
            (kK_prevCmd * (gAppliedVolts - gPrevFeedforward));
  u = clampF(u, kMaxVolts);
  u = gAppliedVolts + clampF(u - gAppliedVolts, kMaxVoltStepPerTick);

  // Stall protection: a command pinned at the rail for too long trips.
  if (fabsFast(u) >= kSaturationVolts) {
    gSaturatedSec += dt;
    if (gSaturatedSec >= kSaturationTripSec) {
      gArmed = false;
      gSaturationLockout = true;
      driveMotors(0);
      return;
    }
  } else {
    gSaturatedSec = 0.0f;
  }

  // Slide the setpoint toward the tilt that needs no position term (takes
  // effect next tick).  Frozen while driving: acceleration is not a balance error.
  if (kBalanceTrimTauSec > 0.0f && !driving) {
    gSetpointDeg -= dt * posTerm / (kK_tilt * kBalanceTrimTauSec);
    gSetpointDeg = gArmTiltDeg + clampF(gSetpointDeg - gArmTiltDeg, kMaxTrimDeg);
  }

  // Advance the observer with the voltage applied over the tick just ended.
  gObsZ += dt * ((kObsInputGain * gAppliedVolts) - (kObsDecay * gObsZ) +
                 (kObsRateGain * rateRad));
  gWheelAngleHat += dt * (wheelRateHat - (kWheelPosLeakPerSec * gWheelAngleHat));
  gAppliedVolts = u;
  gPrevFeedforward = feedforward;

  // ---- Turn: yaw-rate loop -> differential volts (balance keeps priority) ---------
  float diff = 0.0f;
  if (kEnableBluetoothDrive) {
    const float yawRateErr = gTurnRef - (yawRateRawDps(raw) - gYawBiasDps);
    gHeadingErrDeg = clampF(gHeadingErrDeg + yawRateErr * dt, kMaxHeadingErrDeg);
    diff = (kTurnFeedforwardVoltsPerDps * gTurnRef) + (kTurnP * yawRateErr) +
           (kHeadingP * gHeadingErrDeg);
    diff = clampF(clampF(diff, kMaxTurnVolts), kMaxVolts - fabsFast(u));
  }

  // + diff speeds the right wheel up and the left down = turn left.
  driveWheels(voltsToPwm(u + diff), voltsToPwm(u - diff));

#ifdef BWB_DEBUG  // diagnostics only: -DBWB_DEBUG, USB serial 115200, 100 Hz
  // Integers keep the print cheap: tiltErr [cdeg], u [cV], rate [0.1 dps],
  // pwmR, pwmL.
  static uint8_t dbgDiv = 0;
  if (++dbgDiv >= 2) {
    dbgDiv = 0;
    Serial.print(static_cast<int16_t>(tiltErr * 100.0f)); Serial.print(' ');
    Serial.print(static_cast<int16_t>(u * 100.0f)); Serial.print(' ');
    Serial.print(static_cast<int16_t>(rateDps * 10.0f)); Serial.print(' ');
    Serial.print(gLastPwmRight); Serial.print(' ');
    Serial.println(gLastPwmLeft);
  }
#endif
}
