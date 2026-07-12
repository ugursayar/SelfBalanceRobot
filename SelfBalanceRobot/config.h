#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

enum class GyroAxis : uint8_t {
  X,
  Y,
  Z
};

namespace Config {
  constexpr bool BareBalanceFirmware = true;

  constexpr uint8_t GyroPort = 0x06;
  constexpr GyroAxis BalanceGyroAxis = GyroAxis::X;
  constexpr float BalanceGyroRateSign = -1.0f;

  constexpr uint8_t RightEncoderSlot = 1;
  constexpr uint8_t LeftEncoderSlot = 2;

  constexpr unsigned long BalanceLoopMicros =
      BareBalanceFirmware ? 5000UL : 5000UL;
  constexpr unsigned long CommandTimeoutMillis = 1500UL;
  constexpr unsigned long CalibrationMillis = 1200UL;

  constexpr float BalanceKp = BareBalanceFirmware ? 28.0f : 14.0f;
  constexpr float BalanceKi = 0.0f;
  constexpr float BalanceKd = BareBalanceFirmware ? 0.67f : 2.2f;
  constexpr float BalanceRateFilterAlpha =
      BareBalanceFirmware ? 0.55f : 0.30f;
  // Nonlinear gain schedule: proportional gain is scaled to
  // BalanceSmallErrorGainScale at upright and ramps to full kp at
  // BalanceSmallErrorDegrees. Gentle near balance (kills the wobble), strong on
  // a big lean (plus LargeLeanBoost beyond LargeLeanBoostAngleDegrees).
  constexpr float BalanceSmallErrorDegrees = 5.0f;
  constexpr float BalanceSmallErrorGainScale = 0.30f;
  constexpr float WheelSpeedTargetCorrectionDegreesPerRpm =
      0.0f;
  constexpr float MaxWheelSpeedTargetCorrectionDegrees =
      0.0f;
  constexpr float WheelSpeedDampingCommandPerRpm =
      BareBalanceFirmware ? -0.06f : 0.0f;
  constexpr float WheelSpeedDampingMaxAngleErrorDegrees = 4.0f;
  constexpr float TravelHoldTargetDegreesPerWheelDegree = 0.0f;
  constexpr float MaxTravelHoldTargetCorrectionDegrees = 0.5f;
  constexpr float IntegralLimitDegreesSeconds = 30.0f;
  constexpr float MinBalanceBoostAngleDegrees = 0.80f;
  constexpr float LargeLeanBoostAngleDegrees = 2.0f;
  constexpr float LargeLeanBoostCommandPerDegree =
      BareBalanceFirmware ? 0.0f : 6.0f;

  // --- LQR state-feedback controller (selectable alternative to PID) ---
  // Selected exactly the way the Bluetooth channel is: a constexpr flag the
  // balance loop branches on at runtime (dead-code-eliminated for the unused
  // controller).  When true, the loop runs a Linear-Quadratic Regulator control
  // law u = K * x in place of the PID law.  The PID gain schedule, integral
  // term, and large-lean boost do not apply in LQR mode (LQR is linear).
  constexpr bool EnableLqrController = true;
  // Full-state gain row K = [angle, angleRate, wheelPosition, wheelSpeed],
  // expressed in the output (motor-command) sign convention so a positive gain
  // produces a corrective command in the same direction as the PID kp/kd terms.
  // Compute K offline by solving the Riccati equation for the linearized plant
  // (A, B) against the chosen state/effort weights (Q, R) and paste the row in
  // here.  The defaults below seed K from the existing PID tuning with the wheel
  // states unpenalized, so the LQR path reduces to clean angle/rate state
  // feedback and is stable out of the box until a model-derived K is dropped in.
  constexpr float LqrAngleGain = BalanceKp;
  constexpr float LqrAngleRateGain = BalanceKd;
  constexpr float LqrWheelPositionGain = 0.0f;
  constexpr float LqrWheelSpeedGain = 0.0f;
  constexpr float LqrRateFilterAlpha = BalanceRateFilterAlpha;
  // Near-upright softening of the angle gain.  This does NOT cap peak authority:
  // beyond LqrSmallErrorDegrees the full LqrAngleGain applies, so real leans get
  // full power.  Only inside the band is the angle gain scaled toward
  // LqrSmallErrorGainScale at the target (ramping back to full at the edge),
  // which kills the high-frequency shake that micro-corrections excite through
  // the gear backlash (and the gyro noise that shake feeds back).
  // Keep the band NARROW so full power returns quickly, and the center scale LOW
  // so tiny errors barely move the motors.  0.15 => ~15% response at dead center;
  // raise toward 1.0 for more small-angle response, lower toward 0.0 for less.
  constexpr float LqrSmallErrorDegrees = 2.0f;
  constexpr float LqrSmallErrorGainScale = 0.15f;

  constexpr float BalanceAngleTrimDegrees = 0.0f;
  constexpr float MinRuntimeKp = 0.0f;
  constexpr float MaxRuntimeKp = 200.0f;
  constexpr float MinRuntimeKi = 0.0f;
  constexpr float MaxRuntimeKi = 10.0f;
  constexpr float MinRuntimeKd = 0.0f;
  constexpr float MaxRuntimeKd = 20.0f;
  constexpr float MinRuntimeTrimDegrees = -5.0f;
  constexpr float MaxRuntimeTrimDegrees = 5.0f;
  constexpr float FallAngleDegrees = 35.0f;
  constexpr float StillAngleDeltaDegrees = 4.0f;
  constexpr float MaxStartupUprightAngleDegrees = 12.0f;
  constexpr float SafetyCutoffAngleErrorDegrees =
      BareBalanceFirmware ? 8.0f : 0.0f;
  constexpr int16_t SafetyCutoffMotorCommand =
      BareBalanceFirmware ? 180 : 0;
  constexpr unsigned long SafetyCutoffMillis =
      BareBalanceFirmware ? 250UL : 0UL;
  constexpr bool EnableAutoArm = true;
  constexpr uint16_t BalancePointEepromAddress = 0;
  constexpr float AutoArmDefaultBalancePointDegrees =
      BareBalanceFirmware ? 0.55f : 0.70f;
  constexpr float MinPersistedBalancePointDegrees = -12.0f;
  constexpr float MaxPersistedBalancePointDegrees = 12.0f;
  constexpr float AutoArmAngleWindowDegrees =
      BareBalanceFirmware ? 4.0f : 1.0f;
  constexpr float AutoArmMaxRateDegPerSec =
      BareBalanceFirmware ? 20.0f : 4.0f;
  // How long the robot must stay inside the angle/rate window before auto-arm
  // fires.  Held by hand, a long dwell is hard to satisfy (any tremor that
  // briefly breaks the rate limit restarts the count), so keep it short enough
  // to be easy but long enough to confirm it is genuinely being held still.
  constexpr unsigned long AutoArmStillMillis = 500UL;
  constexpr unsigned long AutoArmStopCooldownMillis = 3000UL;

  constexpr unsigned long BalancePointLearningSettleMillis = 1500UL;
  constexpr unsigned long BalancePointLearningStableMillis = 1000UL;
  constexpr unsigned long BalancePointMinWriteIntervalMillis = 30000UL;
  constexpr float BalancePointLearningMaxAngleErrorDegrees = 1.0f;
  constexpr float BalancePointLearningMaxRateDegPerSec = 5.0f;
  constexpr int16_t BalancePointLearningMaxMotorCommand = 45;
  constexpr float BalancePointLearningAlpha = 0.25f;

  constexpr int16_t MotorDeadband = 0;
  constexpr int16_t MinBalanceMotorCommand = BareBalanceFirmware ? 5 : 12;
  constexpr int16_t MaxMotorCommand = BareBalanceFirmware ? 180 : 80;
  constexpr int16_t MotorTestCommand = 30;
  constexpr unsigned long MotorTestMillis = 700UL;

  constexpr bool EnableDebugSerial = false;
  constexpr bool EnableBluetoothTestControl = !BareBalanceFirmware;
  constexpr bool EnableBalancePointLearning = !BareBalanceFirmware;
  constexpr bool EnableBalancePointLearningByDefault = false;
  // Wheel feedback (encoders) is unused by the current control law (speed/travel
  // gains are 0), so keep it OFF: skips per-tick encoder reads AND the encoder
  // interrupts, freeing CPU for the control loop.
  constexpr bool EnableMotorFeedback = false;
  constexpr unsigned long BluetoothBaud = 115200UL;
  constexpr unsigned long BluetoothTelemetryPeriodMillis = 250UL;
  constexpr unsigned long DebugPeriodMillis = 200UL;
  constexpr uint8_t FeedbackFullRefreshPeriodTicks = 5;

  // Ramp the balance target from uprightAngle down to (upright+trim) over this
  // many milliseconds at the start of each balancing session.  Prevents the
  // large initial error spike that occurs when the robot is calibrated well
  // above the true balance point.
  constexpr unsigned long BalanceTargetRampMillis = 1500UL;
}

#endif
