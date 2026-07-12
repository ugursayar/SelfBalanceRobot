#ifndef SENSORS_H
#define SENSORS_H

#include <MeGyro.h>

#include "RobotTypes.h"
#include "config.h"

class Sensors {
public:
  Sensors();

  void begin();
  const SensorFrame& update(unsigned long nowMillis);
  const SensorFrame& current() const;

private:
  float readBalanceAngle() const;
  float readBalanceAngleRate() const;

  MeGyro gyro_;
  SensorFrame frame_;
};

#endif
