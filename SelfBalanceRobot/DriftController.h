#ifndef DRIFT_CONTROLLER_H
#define DRIFT_CONTROLLER_H

#include "RobotTypes.h"

class DriftController {
public:
  DriftController();

  void configure(float positionKp, float speedKp,
                 float maxCorrectionDegrees, bool invertCorrection);
  void reset(const WheelFeedback& feedback);
  float update(const WheelFeedback& feedback);
  float lastCorrectionDegrees() const;

private:
  float positionKp_;
  float speedKp_;
  float maxCorrectionDegrees_;
  bool invertCorrection_;
  float neutralPositionDegrees_;
  float lastCorrectionDegrees_;

  float clampCorrection(float value) const;
};

#endif
