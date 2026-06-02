#include "FeedbackPolicy.h"

void FeedbackPolicy::configure(uint8_t fullRefreshPeriodTicks) {
  fullRefreshPeriodTicks_ =
      fullRefreshPeriodTicks == 0 ? 1 : fullRefreshPeriodTicks;
  ticksUntilFullRefresh_ = 0;
}

MotorFeedbackMode FeedbackPolicy::nextMode(const FeedbackRequest& request) {
  const bool speedFeatureEnabled =
      request.speedTargetCorrectionEnabled || request.speedDampingEnabled;
  if (speedFeatureEnabled) {
    resetCountdown();
    return MotorFeedbackMode::Full;
  }

  if (request.forceFullRefresh) {
    if (ticksUntilFullRefresh_ == 0) {
      resetCountdown();
    } else {
      advanceCountdown();
    }
    return MotorFeedbackMode::Full;
  }

  if (ticksUntilFullRefresh_ == 0) {
    resetCountdown();
    return MotorFeedbackMode::Full;
  }

  advanceCountdown();
  return MotorFeedbackMode::PositionAndPwm;
}

void FeedbackPolicy::resetCountdown() {
  ticksUntilFullRefresh_ =
      fullRefreshPeriodTicks_ > 0 ? fullRefreshPeriodTicks_ - 1 : 0;
}

void FeedbackPolicy::advanceCountdown() {
  if (ticksUntilFullRefresh_ > 0) {
    --ticksUntilFullRefresh_;
  }
}
