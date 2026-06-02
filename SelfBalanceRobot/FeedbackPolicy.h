#ifndef FEEDBACK_POLICY_H
#define FEEDBACK_POLICY_H

#include <stdint.h>

enum class MotorFeedbackMode : uint8_t {
  PositionAndPwm,
  Full
};

struct FeedbackRequest {
  bool speedTargetCorrectionEnabled = false;
  bool speedDampingEnabled = false;
  bool forceFullRefresh = false;
};

class FeedbackPolicy {
public:
  void configure(uint8_t fullRefreshPeriodTicks);
  MotorFeedbackMode nextMode(const FeedbackRequest& request);

private:
  void resetCountdown();
  void advanceCountdown();

  uint8_t fullRefreshPeriodTicks_ = 1;
  uint8_t ticksUntilFullRefresh_ = 0;
};

#endif
