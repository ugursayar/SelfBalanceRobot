#include <cassert>
#include <cstdlib>
#include <iostream>

#include "../../SelfBalanceRobot/config.h"

// NOTE: the PID scalars (kp/kd/alpha/MaxMotorCommand/MinBalanceMotorCommand,
// loop period) are under active on-robot tuning and intentionally NOT pinned
// here, to avoid churn on every tuning step. This guard pins the structural and
// safety decisions that must not silently regress.

static void test_structural_profile() {
  assert(!Config::BareBalanceFirmware);
  assert(!Config::EnableDebugSerial);    // no continuous telemetry in the hot loop
  assert(!Config::EnableMotorFeedback);  // encoders unused -> off for performance
  assert(Config::EnableBluetoothTestControl);
  assert(Config::EnableBalancePointLearning);
  assert(!Config::EnableBalancePointLearningByDefault);
}

static void test_sensor_axis_and_sign() {
  assert(Config::BalanceGyroAxis == GyroAxis::X);
  assert(Config::BalanceGyroRateSign == -1.0f);
}

static void test_gain_schedule_is_active() {
  // Weak proportional response near upright, full kp at a larger lean.
  assert(Config::BalanceSmallErrorDegrees > 0.0f);
  assert(Config::BalanceSmallErrorGainScale >= 0.0f);
  assert(Config::BalanceSmallErrorGainScale < 1.0f);
}

static void test_outer_loops_disabled_for_baseline() {
  assert(Config::WheelSpeedTargetCorrectionDegreesPerRpm == 0.0f);
  assert(Config::MaxWheelSpeedTargetCorrectionDegrees == 0.0f);
  assert(Config::WheelSpeedDampingCommandPerRpm == 0.0f);
  assert(Config::TravelHoldTargetDegreesPerWheelDegree == 0.0f);
}

static void test_recovery_authority_and_safety() {
  assert(Config::FallAngleDegrees == 35.0f);
  assert(Config::SafetyCutoffMillis == 0UL);
  assert(Config::BalanceAngleTrimDegrees == 0.0f);
}

static void test_auto_arm_window_matches_tuning() {
  assert(Config::AutoArmAngleWindowDegrees == 1.0f);
  assert(Config::AutoArmMaxRateDegPerSec == 4.0f);
}

static void test_lqr_controller_is_selectable_and_seeded_from_pid() {
  // EnableLqrController is an actively-toggled selector (like BareBalanceFirmware),
  // so its on/off value is intentionally not pinned here.
  // Default K seeds angle/rate from the PID tuning with wheel states off, so
  // the LQR path reduces to clean angle/rate state feedback out of the box.
  assert(Config::LqrAngleGain == Config::BalanceKp);
  assert(Config::LqrAngleRateGain == Config::BalanceKd);
  assert(Config::LqrWheelPositionGain == 0.0f);
  assert(Config::LqrWheelSpeedGain == 0.0f);
  // Near-upright gain schedule is active so small corrections are softened.
  assert(Config::LqrSmallErrorDegrees > 0.0f);
  assert(Config::LqrSmallErrorGainScale >= 0.0f);
  assert(Config::LqrSmallErrorGainScale < 1.0f);
}

int main() {
  test_structural_profile();
  test_sensor_axis_and_sign();
  test_gain_schedule_is_active();
  test_outer_loops_disabled_for_baseline();
  test_recovery_authority_and_safety();
  test_auto_arm_window_matches_tuning();
  test_lqr_controller_is_selectable_and_seeded_from_pid();

  std::cout << "test_config PASS\n";
  return EXIT_SUCCESS;
}
