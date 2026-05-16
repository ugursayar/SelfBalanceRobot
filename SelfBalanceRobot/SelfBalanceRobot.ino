#include "config.h"
#include "RobotTypes.h"

void setup() {
  Serial.begin(115200);
  if (Config::EnableDebugSerial) {
    Serial.println(F("SelfBalanceRobot skeleton ready"));
  }
}

void loop() {
}
