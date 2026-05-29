#include "AutoArmController.h"
#include "BalanceController.h"
#include "BalancePointLearner.h"
#include "BalancePointStore.h"
#include "CommandReader.h"
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
CommandReader usbCommandReader;
CommandReader bluetoothCommandReader;

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
bool runtimeAutoArmEnabled = Config::EnableAutoArm;
bool balancePointLearningEnabled = true;
bool bluetoothTelemetryEnabled = false;

void readCommands(unsigned long nowMillis);
void readCommandsFrom(CommandReader& reader, Stream& reply,
                      unsigned long nowMillis);
void applyParsedCommand(const ParsedCommand& parsed, Stream& reply);
void applyStopCommand(unsigned long nowMillis);
void clearPersistedBalancePoint(Stream& reply, unsigned long nowMillis);
void setPersistedBalancePoint(float angleDegrees, Stream& reply,
                              unsigned long nowMillis);
void configureAutoArmAndLearning();
void printBalancePointStatus(bool loaded);
void printBalancePointStatusTo(Stream& out);
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
void printStatus(Stream& out, const SensorFrame& frame);
void printMode(Stream& out, RobotMode mode);
void printModeChangeIfNeeded();
void printDebug(const SensorFrame& frame);
void printDebugTo(Stream& out, const SensorFrame& frame);
void updateStatusLed(unsigned long nowMillis);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  usbCommandReader.begin(Serial);
  if (Config::EnableBluetoothTestControl) {
    Serial1.begin(Config::BluetoothBaud);
    bluetoothCommandReader.begin(Serial1);
  }

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
    if (Config::EnableBluetoothTestControl) {
      Serial.println(F("bluetooth-test-control serial1=115200"));
    }
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

void printBalancePointStatusTo(Stream& out) {
  out.print(F("balance-point active="));
  out.print(balancePointStore.balancePointDegrees());
  out.print(F(" stored="));
  out.print(balancePointStore.hasStoredBalancePoint() ? F("yes") : F("no"));
  out.print(F(" writes="));
  out.println(balancePointStore.writeCounter());
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

  readCommands(nowMillis);

  const SensorFrame& frame = sensors.update(nowMillis);
  lastWheelFeedback = motors.updateFeedback();

  const RobotMode previousMode = robotState.mode();
  robotState.update(frame, command);
  if (command.stop) {
    command.stop = false;
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
  if (!runtimeAutoArmEnabled) {
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
    if (Config::EnableBluetoothTestControl && bluetoothTelemetryEnabled) {
      Serial1.print(F("auto-arm balancePoint="));
      Serial1.println(activeBalancePointDegrees);
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
  if (!balancePointLearningEnabled) {
    return;
  }

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
  if (Config::EnableBluetoothTestControl && bluetoothTelemetryEnabled) {
    Serial1.print(F("balance-point saved="));
    Serial1.print(activeBalancePointDegrees);
    Serial1.print(F(" writes="));
    Serial1.println(balancePointStore.writeCounter());
  }
}

void readCommands(unsigned long nowMillis) {
  readCommandsFrom(usbCommandReader, Serial, nowMillis);
  if (Config::EnableBluetoothTestControl) {
    readCommandsFrom(bluetoothCommandReader, Serial1, nowMillis);
  }
}

void readCommandsFrom(CommandReader& reader, Stream& reply,
                      unsigned long nowMillis) {
  ParsedCommand parsed;
  while (reader.readCommand(parsed, nowMillis)) {
    applyParsedCommand(parsed, reply);
  }
}

void applyParsedCommand(const ParsedCommand& parsed, Stream& reply) {
  switch (parsed.action) {
  case ParsedCommandAction::None:
    return;
  case ParsedCommandAction::Invalid:
    reply.println(F("error command"));
    return;
  case ParsedCommandAction::Arm:
    if (!command.stop) {
      command.arm = true;
      command.stop = false;
      command.receivedMillis = parsed.receivedMillis;
      reply.println(F("ok arm"));
    }
    return;
  case ParsedCommandAction::Stop:
    applyStopCommand(parsed.receivedMillis);
    reply.println(F("ok stop"));
    return;
  case ParsedCommandAction::MotorBackward:
    startMotorTest(Config::MotorTestCommand, parsed.receivedMillis);
    reply.println(F("ok m+"));
    return;
  case ParsedCommandAction::MotorForward:
    startMotorTest(-Config::MotorTestCommand, parsed.receivedMillis);
    reply.println(F("ok m-"));
    return;
  case ParsedCommandAction::SetTrim:
    applyRuntimeTrim(parsed.first);
    reply.print(F("ok trim="));
    reply.println(currentTrimDegrees);
    return;
  case ParsedCommandAction::SetPid:
    applyRuntimePid(parsed.first, parsed.second, parsed.third);
    reply.print(F("ok pid kp="));
    reply.print(currentKp);
    reply.print(F(" ki="));
    reply.print(currentKi);
    reply.print(F(" kd="));
    reply.println(currentKd);
    return;
  case ParsedCommandAction::BalancePointQuery:
    printBalancePointStatusTo(reply);
    return;
  case ParsedCommandAction::BalancePointSet:
    setPersistedBalancePoint(parsed.first, reply, parsed.receivedMillis);
    return;
  case ParsedCommandAction::BalancePointClear:
    clearPersistedBalancePoint(reply, parsed.receivedMillis);
    return;
  case ParsedCommandAction::AutoOn:
    runtimeAutoArmEnabled = Config::EnableAutoArm;
    reply.println(runtimeAutoArmEnabled ? F("ok auto=on")
                                        : F("ok auto=disabled-by-config"));
    return;
  case ParsedCommandAction::AutoOff:
    runtimeAutoArmEnabled = false;
    autoArm.suppressUntil(parsed.receivedMillis,
                          Config::AutoArmStopCooldownMillis);
    reply.println(F("ok auto=off"));
    return;
  case ParsedCommandAction::LearnOn:
    balancePointLearningEnabled = true;
    balancePointLearner.reset(balancePointStore.balancePointDegrees(),
                              parsed.receivedMillis);
    reply.println(F("ok learn=on"));
    return;
  case ParsedCommandAction::LearnOff:
    balancePointLearningEnabled = false;
    reply.println(F("ok learn=off"));
    return;
  case ParsedCommandAction::Status:
    printStatus(reply, lastFrame);
    return;
  case ParsedCommandAction::TelemetryOn:
    bluetoothTelemetryEnabled = true;
    reply.println(F("ok telem=on"));
    return;
  case ParsedCommandAction::TelemetryOff:
    bluetoothTelemetryEnabled = false;
    reply.println(F("ok telem=off"));
    return;
  }
}

void applyStopCommand(unsigned long nowMillis) {
  command.arm = false;
  command.stop = true;
  command.receivedMillis = nowMillis;
  motorTestUntilMillis = 0;
  autoArm.suppressUntil(nowMillis, Config::AutoArmStopCooldownMillis);
  balanceSessionUsesPersistedPoint = false;
}

void clearPersistedBalancePoint(Stream& reply, unsigned long nowMillis) {
  applyStopCommand(nowMillis);
  balancePointStore.clearBalancePoint(Config::AutoArmDefaultBalancePointDegrees);
  activeBalancePointDegrees = balancePointStore.balancePointDegrees();
  autoArm.setTargetBalancePoint(activeBalancePointDegrees);
  balancePointLearner.reset(activeBalancePointDegrees, nowMillis);
  reply.print(F("ok bp-clear default="));
  reply.println(activeBalancePointDegrees);
}

void setPersistedBalancePoint(float angleDegrees, Stream& reply,
                              unsigned long nowMillis) {
  if (!balancePointStore.saveBalancePoint(angleDegrees)) {
    reply.println(F("error bp-range"));
    return;
  }
  activeBalancePointDegrees = balancePointStore.balancePointDegrees();
  autoArm.setTargetBalancePoint(activeBalancePointDegrees);
  balancePointLearner.reset(activeBalancePointDegrees, nowMillis);
  reply.print(F("ok bp="));
  reply.print(activeBalancePointDegrees);
  reply.print(F(" writes="));
  reply.println(balancePointStore.writeCounter());
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

void printStatus(Stream& out, const SensorFrame& frame) {
  out.print(F("status mode="));
  printMode(out, robotState.mode());
  out.print(F(" angle="));
  out.print(frame.angleDegrees);
  out.print(F(" target="));
  out.print(lastTargetAngle);
  out.print(F(" trim="));
  out.print(currentTrimDegrees);
  out.print(F(" bp="));
  out.print(activeBalancePointDegrees);
  out.print(F(" stored="));
  out.print(balancePointStore.hasStoredBalancePoint() ? F("yes") : F("no"));
  out.print(F(" rate="));
  out.print(balance.lastMeasuredAngleRateDegreesPerSecond());
  out.print(F(" raw="));
  out.print(lastRawBalanceOutput);
  out.print(F(" balance="));
  out.print(lastBalanceOutput);
  out.print(F(" speed="));
  out.print(lastWheelFeedback.averageSpeedRpm);
  out.print(F(" pos="));
  out.print(lastWheelFeedback.averagePositionDegrees);
  out.print(F(" hold="));
  out.print(lastTravelHoldTargetCorrection);
  out.print(F(" left="));
  out.print(lastMotorOutput.left);
  out.print(F(" right="));
  out.print(lastMotorOutput.right);
  out.print(F(" kp="));
  out.print(currentKp);
  out.print(F(" ki="));
  out.print(currentKi);
  out.print(F(" kd="));
  out.println(currentKd);
}

void printMode(Stream& out, RobotMode mode) {
  switch (mode) {
  case RobotMode::Disarmed:
    out.print(F("disarmed"));
    break;
  case RobotMode::Calibrating:
    out.print(F("calibrating"));
    break;
  case RobotMode::Balancing:
    out.print(F("balancing"));
    break;
  case RobotMode::Fault:
    out.print(F("fault"));
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
  printMode(Serial, mode);
  Serial.print(F(" upright="));
  Serial.print(robotState.uprightAngleDegrees());
  Serial.print(F(" calibRange="));
  Serial.println(robotState.calibrationRangeDegrees());
}

void printDebug(const SensorFrame& frame) {
  if (!Config::EnableDebugSerial && !bluetoothTelemetryEnabled) {
    return;
  }

  if (frame.nowMillis - lastDebugMillis < Config::DebugPeriodMillis) {
    return;
  }
  lastDebugMillis = frame.nowMillis;

  if (Config::EnableDebugSerial) {
    printDebugTo(Serial, frame);
  }
  if (Config::EnableBluetoothTestControl && bluetoothTelemetryEnabled) {
    printDebugTo(Serial1, frame);
  }
}

void printDebugTo(Stream& out, const SensorFrame& frame) {
  out.print(F("mode="));
  printMode(out, robotState.mode());
  out.print(F(" loopUs="));
  out.print(lastLoopMicros);
  out.print(F(" angle="));
  out.print(frame.angleDegrees);
  out.print(F(" upright="));
  out.print(robotState.uprightAngleDegrees());
  out.print(F(" trim="));
  out.print(currentTrimDegrees);
  out.print(F(" target="));
  out.print(lastTargetAngle);
  out.print(F(" err="));
  out.print(balance.lastErrorDegrees());
  out.print(F(" rate="));
  out.print(balance.lastMeasuredAngleRateDegreesPerSecond());
  out.print(F(" raw="));
  out.print(lastRawBalanceOutput);
  out.print(F(" balance="));
  out.print(lastBalanceOutput);
  out.print(F(" speed="));
  out.print(lastWheelFeedback.averageSpeedRpm);
  out.print(F(" pos="));
  out.print(lastWheelFeedback.averagePositionDegrees);
  out.print(F(" hold="));
  out.print(lastTravelHoldTargetCorrection);
  out.print(F(" left="));
  out.print(lastMotorOutput.left);
  out.print(F(" right="));
  out.print(lastMotorOutput.right);
  out.print(F(" lpwm="));
  out.print(lastWheelFeedback.leftPwm);
  out.print(F(" rpwm="));
  out.print(lastWheelFeedback.rightPwm);
  out.print(F(" kp="));
  out.print(currentKp);
  out.print(F(" kd="));
  out.println(currentKd);
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
