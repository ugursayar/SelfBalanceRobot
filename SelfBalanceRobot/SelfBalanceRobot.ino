#include "AutoArmController.h"
#include "BalanceController.h"
#include "BalancePointLearner.h"
#include "BalancePointStore.h"
#include "EepromByteStorage.h"
#include "Motors.h"
#include "RobotState.h"
#include "Sensors.h"
#include "config.h"

#include <MeMegaPi.h>
#include <string.h>

Sensors sensors;
Motors motors;
BalanceController balance;
RobotState robotState;
EepromByteStorage eepromStorage;
BalancePointStore balancePointStore(eepromStorage,
                                    Config::BalancePointEepromAddress);
AutoArmController autoArm;
BalancePointLearner balancePointLearner;

unsigned long lastBalanceMicros = 0;
unsigned long lastDebugMillis = 0;
unsigned long lastLoopMicros = 0;
unsigned long motorTestUntilMillis = 0;
SensorFrame lastFrame;
ControlCommand command;
MotorCommand lastMotorOutput;
WheelFeedback lastWheelFeedback;
float lastTargetAngle = 0.0f;
int16_t lastRawBalanceOutput = 0;
int16_t lastBalanceOutput = 0;
float lastTravelHoldTargetCorrection = 0.0f;
RobotMode lastReportedMode = RobotMode::Disarmed;
float currentTrimDegrees = Config::BalanceAngleTrimDegrees;
float currentKp = Config::BalanceKp;
float currentKi = Config::BalanceKi;
float currentKd = Config::BalanceKd;
bool statusLedOn = false;
unsigned long lastStatusLedToggleMillis = 0;
unsigned long balancingStartMillis = 0;
bool balanceSessionUsesPersistedPoint = false;
float activeBalancePointDegrees = Config::AutoArmDefaultBalancePointDegrees;

void readUsbCommand(unsigned long nowMillis);
void configureAutoArmAndLearning();
void printBalancePointStatus(bool loaded);
void handleAutoArm(const SensorFrame& frame, RobotMode modeBeforeAutoArm);
bool manualArmAttemptIsFresh(unsigned long nowMillis);
void updateBalancePointLearning(const SensorFrame& frame,
                                float baseTargetDegrees,
                                int16_t balanceOutput,
                                unsigned long nowMillis);
void applyRuntimePid(float kp, float ki, float kd);
void applyRuntimeTrim(float trimDegrees);
void startMotorTest(int16_t output, unsigned long nowMillis);
float baseBalanceTargetDegrees();
float clampWheelSpeedTargetCorrection(float correctionDegrees);
float clampTravelHoldTargetCorrection(float correctionDegrees);
int16_t clampMotorCommand(float command);
int16_t applyLargeLeanBoost(int16_t balanceOutput, float angleError);
int16_t applyMinimumBalanceCommand(int16_t balanceOutput, float angleError);
void printMode(RobotMode mode);
void printModeChangeIfNeeded();
void printDebug(const SensorFrame& frame);
void updateStatusLed(unsigned long nowMillis);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);

  sensors.begin();
  motors.begin();

  balance.setTunings(Config::BalanceKp, Config::BalanceKi, Config::BalanceKd);
  balance.setIntegralLimit(Config::IntegralLimitDegreesSeconds);
  balance.setRateFilter(Config::BalanceRateFilterAlpha);
  balance.setOutputLimit(Config::MaxMotorCommand);

  configureAutoArmAndLearning();

  robotState.configure(Config::FallAngleDegrees,
                       Config::StillAngleDeltaDegrees,
                       Config::CalibrationMillis,
                       Config::CommandTimeoutMillis);

  lastBalanceMicros = micros();

  if (Config::EnableDebugSerial) {
    Serial.println(F("SelfBalanceRobot balance-only ready. Send arm or stop."));
  }
}

void configureAutoArmAndLearning() {
  balancePointStore.configure(Config::MinPersistedBalancePointDegrees,
                              Config::MaxPersistedBalancePointDegrees);
  const bool loaded =
      balancePointStore.begin(Config::AutoArmDefaultBalancePointDegrees);
  activeBalancePointDegrees = balancePointStore.balancePointDegrees();

  autoArm.configure(Config::AutoArmAngleWindowDegrees,
                    Config::AutoArmMaxRateDegPerSec,
                    Config::AutoArmStillMillis);
  autoArm.setTargetBalancePoint(activeBalancePointDegrees);

  balancePointLearner.configure(Config::BalancePointLearningSettleMillis,
                                Config::BalancePointLearningStableMillis,
                                Config::BalancePointMinWriteIntervalMillis,
                                Config::BalancePointLearningMaxAngleErrorDegrees,
                                Config::BalancePointLearningMaxRateDegPerSec,
                                Config::BalancePointLearningMaxMotorCommand,
                                Config::BalancePointLearningAlpha);
  printBalancePointStatus(loaded);
}

void printBalancePointStatus(bool loaded) {
  if (!Config::EnableDebugSerial) {
    return;
  }

  Serial.print(F("balance-point "));
  Serial.print(loaded ? F("loaded=") : F("default="));
  Serial.print(balancePointStore.balancePointDegrees());
  Serial.print(F(" writes="));
  Serial.println(balancePointStore.writeCounter());
}

void loop() {
  const unsigned long nowMicros = micros();
  const unsigned long elapsedMicros = nowMicros - lastBalanceMicros;
  if (elapsedMicros < Config::BalanceLoopMicros) {
    return;
  }
  lastBalanceMicros = nowMicros;
  lastLoopMicros = elapsedMicros;

  const float dtSeconds = static_cast<float>(elapsedMicros) * 0.000001f;
  const unsigned long nowMillis = millis();

  readUsbCommand(nowMillis);

  const SensorFrame& frame = sensors.update(nowMillis);
  lastWheelFeedback = motors.updateFeedback();

  const RobotMode previousMode = robotState.mode();
  robotState.update(frame, command);
  if (command.stop) {
    command.stop = false;
    autoArm.suppressUntil(nowMillis, Config::AutoArmStopCooldownMillis);
    balanceSessionUsesPersistedPoint = false;
  }
  handleAutoArm(frame, previousMode);
  const RobotMode currentMode = robotState.mode();
  if (previousMode != currentMode && currentMode == RobotMode::Balancing) {
    motors.resetTravel();
    lastWheelFeedback = motors.updateFeedback();
    balance.reset();
    lastBalanceOutput = 0;
    balancingStartMillis = nowMillis;
    if (!balanceSessionUsesPersistedPoint) {
      activeBalancePointDegrees =
          robotState.uprightAngleDegrees() + currentTrimDegrees;
    }
    balancePointLearner.reset(balancePointStore.balancePointDegrees(),
                              nowMillis);
  }

  if (robotState.motorsEnabled()) {
    const float speedCorrection = clampWheelSpeedTargetCorrection(
        lastWheelFeedback.averageSpeedRpm *
        Config::WheelSpeedTargetCorrectionDegreesPerRpm);
    lastTravelHoldTargetCorrection = clampTravelHoldTargetCorrection(
        lastWheelFeedback.averagePositionDegrees *
        Config::TravelHoldTargetDegreesPerWheelDegree);
    // Ramp the target from uprightAngle to (upright+trim) over
    // BalanceTargetRampMillis to avoid large initial error spikes.
    const float baseTarget = baseBalanceTargetDegrees();
    const float finalTarget = baseTarget + speedCorrection +
                              lastTravelHoldTargetCorrection;
    const unsigned long rampMs = Config::BalanceTargetRampMillis;
    const unsigned long elapsed = nowMillis - balancingStartMillis;
    const float rampStartTarget = balanceSessionUsesPersistedPoint
                                      ? baseTarget
                                      : robotState.uprightAngleDegrees();
    if (elapsed < rampMs) {
      const float rampFraction = static_cast<float>(elapsed) /
                                 static_cast<float>(rampMs);
      lastTargetAngle = rampStartTarget +
                        rampFraction * (finalTarget - rampStartTarget);
    } else {
      lastTargetAngle = finalTarget;
    }
    balance.setTargetAngle(lastTargetAngle);

    int16_t balanceOutput = balance.update(frame.angleDegrees,
                                           frame.angleRateDegPerSec,
                                           dtSeconds);
    lastRawBalanceOutput = balanceOutput;
    balanceOutput =
        applyMinimumBalanceCommand(balanceOutput,
                                   lastTargetAngle - frame.angleDegrees);
    balanceOutput =
        applyLargeLeanBoost(balanceOutput,
                            lastTargetAngle - frame.angleDegrees);
    const float angleError = lastTargetAngle - frame.angleDegrees;
    const float absAngleError = angleError < 0.0f ? -angleError : angleError;
    if (absAngleError <= Config::WheelSpeedDampingMaxAngleErrorDegrees) {
      balanceOutput = clampMotorCommand(
          static_cast<float>(balanceOutput) -
          (lastWheelFeedback.averageSpeedRpm *
           Config::WheelSpeedDampingCommandPerRpm));
    }

    lastBalanceOutput = balanceOutput;
    updateBalancePointLearning(frame, baseTarget, balanceOutput, nowMillis);
    lastMotorOutput.left = balanceOutput;
    lastMotorOutput.right = balanceOutput;
    motors.write(lastMotorOutput);
  } else if (static_cast<long>(motorTestUntilMillis - nowMillis) > 0) {
    motors.write(lastMotorOutput);
  } else {
    balance.reset();
    motors.stop();
    lastTargetAngle = robotState.uprightAngleDegrees();
    lastRawBalanceOutput = 0;
    lastBalanceOutput = 0;
    lastTravelHoldTargetCorrection = 0.0f;
    lastMotorOutput = MotorCommand();
  }

  lastFrame = frame;
  updateStatusLed(nowMillis);
  printModeChangeIfNeeded();
  printDebug(lastFrame);
}

void handleAutoArm(const SensorFrame& frame, RobotMode modeBeforeAutoArm) {
  if (!Config::EnableAutoArm) {
    return;
  }

  if (modeBeforeAutoArm != RobotMode::Disarmed ||
      robotState.mode() != RobotMode::Disarmed ||
      manualArmAttemptIsFresh(frame.nowMillis) || command.stop ||
      static_cast<long>(motorTestUntilMillis - frame.nowMillis) > 0) {
    autoArm.reset();
    return;
  }

  if (!autoArm.update(frame)) {
    return;
  }

  activeBalancePointDegrees = balancePointStore.balancePointDegrees();
  if (robotState.startBalancingAt(activeBalancePointDegrees)) {
    balanceSessionUsesPersistedPoint = true;
    if (Config::EnableDebugSerial) {
      Serial.print(F("auto-arm balancePoint="));
      Serial.println(activeBalancePointDegrees);
    }
  }
}

bool manualArmAttemptIsFresh(unsigned long nowMillis) {
  return command.arm &&
         nowMillis - command.receivedMillis <= Config::CommandTimeoutMillis;
}

void updateBalancePointLearning(const SensorFrame& frame,
                                float baseTargetDegrees,
                                int16_t balanceOutput,
                                unsigned long nowMillis) {
  const BalanceLearningResult result =
      balancePointLearner.update(frame, baseTargetDegrees, balanceOutput,
                                 nowMillis);
  if (!result.shouldSave) {
    return;
  }

  if (!balancePointStore.saveBalancePoint(result.balancePointDegrees)) {
    return;
  }

  activeBalancePointDegrees = balancePointStore.balancePointDegrees();
  autoArm.setTargetBalancePoint(activeBalancePointDegrees);
  if (Config::EnableDebugSerial) {
    Serial.print(F("balance-point saved="));
    Serial.print(activeBalancePointDegrees);
    Serial.print(F(" writes="));
    Serial.println(balancePointStore.writeCounter());
  }
}

void readUsbCommand(unsigned long nowMillis) {
  static char buffer[32];
  static uint8_t length = 0;

  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());
    if (incoming == '\r' || incoming == '\n') {
      buffer[length] = '\0';
      if (length > 0) {
        if (strcmp(buffer, "arm") == 0) {
          command.arm = true;
          command.stop = false;
          command.receivedMillis = nowMillis;
        } else if (strcmp(buffer, "stop") == 0) {
          command.arm = false;
          command.stop = true;
          command.receivedMillis = nowMillis;
          motorTestUntilMillis = 0;
          autoArm.suppressUntil(nowMillis, Config::AutoArmStopCooldownMillis);
          balanceSessionUsesPersistedPoint = false;
        } else if (strcmp(buffer, "m+") == 0) {
          startMotorTest(Config::MotorTestCommand, nowMillis);
        } else if (strcmp(buffer, "m-") == 0) {
          startMotorTest(-Config::MotorTestCommand, nowMillis);
        } else if (strncmp(buffer, "trim ", 5) == 0) {
          applyRuntimeTrim(atof(buffer + 5));
        } else if (strncmp(buffer, "pid ", 4) == 0) {
          char* p = buffer + 4;
          const float kp = atof(p);
          char* sp1 = strchr(p, ' ');
          if (sp1 != nullptr) {
            const float ki = atof(++sp1);
            char* sp2 = strchr(sp1, ' ');
            if (sp2 != nullptr) {
              applyRuntimePid(kp, ki, atof(++sp2));
            }
          }
        }
      }
      length = 0;
    } else if (length < sizeof(buffer) - 1) {
      buffer[length++] = incoming >= 'A' && incoming <= 'Z'
                             ? static_cast<char>(incoming + ('a' - 'A'))
                             : incoming;
    }
  }
}

void applyRuntimePid(float kp, float ki, float kd) {
  // clamp to safe ranges before touching the controller
  if (kp < Config::MinRuntimeKp) kp = Config::MinRuntimeKp;
  if (kp > Config::MaxRuntimeKp) kp = Config::MaxRuntimeKp;
  if (ki < Config::MinRuntimeKi) ki = Config::MinRuntimeKi;
  if (ki > Config::MaxRuntimeKi) ki = Config::MaxRuntimeKi;
  if (kd < Config::MinRuntimeKd) kd = Config::MinRuntimeKd;
  if (kd > Config::MaxRuntimeKd) kd = Config::MaxRuntimeKd;
  currentKp = kp;
  currentKi = ki;
  currentKd = kd;
  balance.setTunings(currentKp, currentKi, currentKd);
  if (Config::EnableDebugSerial) {
    Serial.print(F("pid-updated kp="));
    Serial.print(currentKp);
    Serial.print(F(" ki="));
    Serial.print(currentKi);
    Serial.print(F(" kd="));
    Serial.println(currentKd);
  }
}

void applyRuntimeTrim(float trimDegrees) {
  if (trimDegrees < Config::MinRuntimeTrimDegrees)
    trimDegrees = Config::MinRuntimeTrimDegrees;
  if (trimDegrees > Config::MaxRuntimeTrimDegrees)
    trimDegrees = Config::MaxRuntimeTrimDegrees;
  currentTrimDegrees = trimDegrees;
  if (Config::EnableDebugSerial) {
    Serial.print(F("trim-updated="));
    Serial.println(currentTrimDegrees);
  }
}

void startMotorTest(int16_t output, unsigned long nowMillis) {
  if (robotState.mode() != RobotMode::Disarmed) {
    return;
  }

  lastMotorOutput.left = output;
  lastMotorOutput.right = output;
  motorTestUntilMillis = nowMillis + Config::MotorTestMillis;
}

float baseBalanceTargetDegrees() {
  if (balanceSessionUsesPersistedPoint) {
    return activeBalancePointDegrees;
  }
  return robotState.uprightAngleDegrees() + currentTrimDegrees;
}

float clampWheelSpeedTargetCorrection(float correctionDegrees) {
  const float limit = Config::MaxWheelSpeedTargetCorrectionDegrees;
  if (correctionDegrees > limit) {
    return limit;
  }
  if (correctionDegrees < -limit) {
    return -limit;
  }
  return correctionDegrees;
}

float clampTravelHoldTargetCorrection(float correctionDegrees) {
  const float limit = Config::MaxTravelHoldTargetCorrectionDegrees;
  if (correctionDegrees > limit) {
    return limit;
  }
  if (correctionDegrees < -limit) {
    return -limit;
  }
  return correctionDegrees;
}

int16_t clampMotorCommand(float command) {
  if (command > Config::MaxMotorCommand) {
    return Config::MaxMotorCommand;
  }
  if (command < -Config::MaxMotorCommand) {
    return -Config::MaxMotorCommand;
  }
  return static_cast<int16_t>(command);
}

int16_t applyLargeLeanBoost(int16_t balanceOutput, float angleError) {
  const float absAngleError = angleError < 0.0f ? -angleError : angleError;
  if (absAngleError <= Config::LargeLeanBoostAngleDegrees) {
    return balanceOutput;
  }

  const float extra =
      (absAngleError - Config::LargeLeanBoostAngleDegrees) *
      Config::LargeLeanBoostCommandPerDegree;
  const float correctionDirection = angleError < 0.0f ? 1.0f : -1.0f;
  return clampMotorCommand(static_cast<float>(balanceOutput) +
                           (correctionDirection * extra));
}

int16_t applyMinimumBalanceCommand(int16_t balanceOutput, float angleError) {
  if (angleError < 0.0f) {
    angleError = -angleError;
  }
  if (angleError < Config::MinBalanceBoostAngleDegrees) {
    return balanceOutput;
  }

  const int16_t minimum = Config::MinBalanceMotorCommand;
  if (minimum <= 0) {
    return balanceOutput;
  }

  if (balanceOutput > 0 && balanceOutput < minimum) {
    return minimum;
  }
  if (balanceOutput < 0 && balanceOutput > -minimum) {
    return static_cast<int16_t>(-minimum);
  }
  return balanceOutput;
}

void printMode(RobotMode mode) {
  switch (mode) {
  case RobotMode::Disarmed:
    Serial.print(F("disarmed"));
    break;
  case RobotMode::Calibrating:
    Serial.print(F("calibrating"));
    break;
  case RobotMode::Balancing:
    Serial.print(F("balancing"));
    break;
  case RobotMode::Fault:
    Serial.print(F("fault"));
    break;
  }
}

void printModeChangeIfNeeded() {
  const RobotMode mode = robotState.mode();
  if (mode == lastReportedMode) {
    return;
  }

  lastReportedMode = mode;
  if (!Config::EnableDebugSerial) {
    return;
  }

  Serial.print(F("mode-change="));
  printMode(mode);
  Serial.print(F(" upright="));
  Serial.print(robotState.uprightAngleDegrees());
  Serial.print(F(" calibRange="));
  Serial.println(robotState.calibrationRangeDegrees());
}

void printDebug(const SensorFrame& frame) {
  if (!Config::EnableDebugSerial) {
    return;
  }

  if (frame.nowMillis - lastDebugMillis < Config::DebugPeriodMillis) {
    return;
  }
  lastDebugMillis = frame.nowMillis;

  Serial.print(F("mode="));
  printMode(robotState.mode());
  Serial.print(F(" loopUs="));
  Serial.print(lastLoopMicros);
  Serial.print(F(" angle="));
  Serial.print(frame.angleDegrees);
  Serial.print(F(" upright="));
  Serial.print(robotState.uprightAngleDegrees());
  Serial.print(F(" trim="));
  Serial.print(currentTrimDegrees);
  Serial.print(F(" target="));
  Serial.print(lastTargetAngle);
  Serial.print(F(" err="));
  Serial.print(balance.lastErrorDegrees());
  Serial.print(F(" rate="));
  Serial.print(balance.lastMeasuredAngleRateDegreesPerSecond());
  Serial.print(F(" raw="));
  Serial.print(lastRawBalanceOutput);
  Serial.print(F(" balance="));
  Serial.print(lastBalanceOutput);
  Serial.print(F(" speed="));
  Serial.print(lastWheelFeedback.averageSpeedRpm);
  Serial.print(F(" pos="));
  Serial.print(lastWheelFeedback.averagePositionDegrees);
  Serial.print(F(" hold="));
  Serial.print(lastTravelHoldTargetCorrection);
  Serial.print(F(" left="));
  Serial.print(lastMotorOutput.left);
  Serial.print(F(" right="));
  Serial.print(lastMotorOutput.right);
  Serial.print(F(" lpwm="));
  Serial.print(lastWheelFeedback.leftPwm);
  Serial.print(F(" rpwm="));
  Serial.print(lastWheelFeedback.rightPwm);
  Serial.print(F(" kp="));
  Serial.print(currentKp);
  Serial.print(F(" kd="));
  Serial.println(currentKd);
}

void updateStatusLed(unsigned long nowMillis) {
  switch (robotState.mode()) {
  case RobotMode::Disarmed:
    statusLedOn = false;
    digitalWrite(LED_BUILTIN, LOW);
    break;
  case RobotMode::Calibrating:
    if (nowMillis - lastStatusLedToggleMillis >= 100UL) {
      statusLedOn = !statusLedOn;
      lastStatusLedToggleMillis = nowMillis;
      digitalWrite(LED_BUILTIN, statusLedOn ? HIGH : LOW);
    }
    break;
  case RobotMode::Balancing:
    statusLedOn = true;
    digitalWrite(LED_BUILTIN, HIGH);
    break;
  case RobotMode::Fault:
    if (nowMillis - lastStatusLedToggleMillis >= 500UL) {
      statusLedOn = !statusLedOn;
      lastStatusLedToggleMillis = nowMillis;
      digitalWrite(LED_BUILTIN, statusLedOn ? HIGH : LOW);
    }
    break;
  }
}
