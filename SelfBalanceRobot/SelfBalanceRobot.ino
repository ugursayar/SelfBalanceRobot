#include "AutoArmController.h"
#include "BalanceController.h"
#include "BalancePipeline.h"
#include "BalancePointLearner.h"
#include "BalancePointStore.h"
#include "BluetoothSerialPort.h"
#include "CommandReader.h"
#include "EepromByteStorage.h"
#include "FeedbackPolicy.h"
#include "LqrController.h"
#include "MotorOutputLatch.h"
#include "Motors.h"
#include "ResetDiagnostics.h"
#include "RobotState.h"
#include "RuntimeStats.h"
#include "SafetyCutoff.h"
#include "Sensors.h"
#include "TelemetryFormatter.h"
#include "config.h"

#include <MeMegaPi.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <string.h>

uint8_t resetCauseRaw __attribute__((section(".noinit")));
void captureResetCause() __attribute__((naked)) __attribute__((section(".init3")));
void captureResetCause() {
  resetCauseRaw = ResetDiagnostics::sanitize(MCUSR);
  MCUSR = 0;
  wdt_disable();
}

Sensors sensors;
Motors motors;
BalanceController balance;
LqrController lqr;
BalancePipeline balancePipeline;
FeedbackPolicy feedbackPolicy;
RobotState robotState;
EepromByteStorage eepromStorage;
BalancePointStore balancePointStore(eepromStorage,
                                    Config::BalancePointEepromAddress);
AutoArmController autoArm;
BalancePointLearner balancePointLearner;
CommandReader usbCommandReader;
CommandReader bluetoothCommandReader;
RuntimeStats runtimeStats;
MotorOutputLatch motorOutputLatch;
SafetyCutoff safetyCutoff;

unsigned long lastBalanceMicros = 0;
unsigned long lastDebugMillis = 0;
unsigned long lastBluetoothTelemetryMillis = 0;
unsigned long lastLoopMicros = 0;
unsigned long motorTestUntilMillis = 0;
unsigned long manualCommandSuppressedUntilMillis = 0;
SensorFrame lastFrame;
ControlCommand command;
MotorCommand lastMotorOutput;
WheelFeedback lastWheelFeedback;
StopReason lastStopReason = StopReason::None;
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
bool balancePointSavePending = false;
float pendingBalancePointDegrees = 0.0f;
bool runtimeAutoArmEnabled = Config::EnableAutoArm;
bool balancePointLearningEnabled =
    Config::EnableBalancePointLearningByDefault;
bool bluetoothTelemetryEnabled = false;

void readCommands(unsigned long nowMillis);
bool readCommandsFrom(CommandReader& reader, Stream& reply,
                      unsigned long nowMillis);
bool applyParsedCommand(const ParsedCommand& parsed, Stream& reply);
void applyStopCommand(unsigned long nowMillis,
                      StopReason reason = StopReason::Command);
void resetCommandInputs();
void drainCommandStream(Stream& stream);
void clearPersistedBalancePoint(Stream& reply, unsigned long nowMillis);
void setPersistedBalancePoint(float angleDegrees, Stream& reply,
                              unsigned long nowMillis);
void configureAutoArmAndLearning();
void printBalancePointStatus(bool loaded);
void printBalancePointStatusTo(Stream& out);
TelemetrySnapshot buildTelemetrySnapshot(const SensorFrame& frame);
void handleAutoArm(const SensorFrame& frame, RobotMode modeBeforeAutoArm);
bool motorTestActive(unsigned long nowMillis);
bool manualCommandsSuppressed(unsigned long nowMillis);
bool manualArmAttemptIsFresh(unsigned long nowMillis);
void updateBalancePointLearning(const SensorFrame& frame,
                                float baseTargetDegrees,
                                int16_t balanceOutput,
                                unsigned long nowMillis);
void flushPendingBalancePoint();
void applyRuntimePid(float kp, float ki, float kd);
void applyRuntimeTrim(float trimDegrees);
bool startMotorTest(int16_t output, unsigned long nowMillis);
void printStatus(Stream& out, const SensorFrame& frame);
void printMode(Stream& out, RobotMode mode);
void printModeChangeIfNeeded();
void printDebug(const SensorFrame& frame);
void printDebugTo(Stream& out, const SensorFrame& frame);
void printBluetoothTelemetryTo(Stream& out, const SensorFrame& frame);
void setStatusLed(bool on);
bool updateReasonBlink(unsigned long nowMillis, StopReason reason);
void updateStatusLed(unsigned long nowMillis);

void setup() {
  resetCauseRaw = ResetDiagnostics::sanitize(resetCauseRaw);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  usbCommandReader.begin(Serial);
  if (Config::EnableBluetoothTestControl) {
    ROBOT_BLUETOOTH_SERIAL.begin(Config::BluetoothBaud);
    bluetoothCommandReader.begin(ROBOT_BLUETOOTH_SERIAL);
  }

  sensors.begin();
  motors.begin();

  balance.setTunings(Config::BalanceKp, Config::BalanceKi, Config::BalanceKd);
  balance.setIntegralLimit(Config::IntegralLimitDegreesSeconds);
  balance.setGainSchedule(Config::BalanceSmallErrorDegrees,
                          Config::BalanceSmallErrorGainScale);
  balance.setRateFilter(Config::BalanceRateFilterAlpha);
  balance.setOutputLimit(Config::MaxMotorCommand);
  lqr.setGains(Config::LqrAngleGain, Config::LqrAngleRateGain,
               Config::LqrWheelPositionGain, Config::LqrWheelSpeedGain);
  lqr.setGainSchedule(Config::LqrSmallErrorDegrees,
                      Config::LqrSmallErrorGainScale);
  lqr.setRateFilter(Config::LqrRateFilterAlpha);
  lqr.setOutputLimit(Config::MaxMotorCommand);
  safetyCutoff.configure(Config::SafetyCutoffAngleErrorDegrees,
                         Config::SafetyCutoffMotorCommand,
                         Config::SafetyCutoffMillis);

  feedbackPolicy.configure(Config::FeedbackFullRefreshPeriodTicks);
  configureAutoArmAndLearning();

  robotState.configure(Config::FallAngleDegrees,
                       Config::StillAngleDeltaDegrees,
                       Config::CalibrationMillis,
                       Config::CommandTimeoutMillis);

  lastBalanceMicros = micros();

  if (Config::EnableDebugSerial) {
    Serial.println(F("SelfBalanceRobot balance-only ready. Send arm or stop."));
    if (Config::EnableBluetoothTestControl) {
      Serial.print(F("bluetooth-test-control serial="));
      Serial.print(RobotBluetoothSerial::Name);
      Serial.print(F(" baud="));
      Serial.println(Config::BluetoothBaud);
    }
  }
}

void configureAutoArmAndLearning() {
  balancePointStore.configure(Config::MinPersistedBalancePointDegrees,
                              Config::MaxPersistedBalancePointDegrees);
  const bool loaded = Config::EnableBalancePointLearning &&
      balancePointStore.begin(Config::AutoArmDefaultBalancePointDegrees);
  activeBalancePointDegrees =
      Config::EnableBalancePointLearning
          ? balancePointStore.balancePointDegrees()
          : Config::AutoArmDefaultBalancePointDegrees;

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
  out.print(activeBalancePointDegrees);
  out.print(F(" store="));
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
  const unsigned long workStartMicros = micros();

  const float dtSeconds = static_cast<float>(elapsedMicros) * 0.000001f;
  const unsigned long nowMillis = millis();

  readCommands(nowMillis);

  const SensorFrame& frame = sensors.update(nowMillis);
  if (Config::EnableMotorFeedback) {
    FeedbackRequest feedbackRequest;
    feedbackRequest.speedTargetCorrectionEnabled =
        Config::WheelSpeedTargetCorrectionDegreesPerRpm != 0.0f ||
        Config::MaxWheelSpeedTargetCorrectionDegrees != 0.0f;
    feedbackRequest.speedDampingEnabled =
        Config::WheelSpeedDampingCommandPerRpm != 0.0f;
    feedbackRequest.forceFullRefresh = bluetoothTelemetryEnabled;
    const MotorFeedbackMode feedbackMode =
        feedbackPolicy.nextMode(feedbackRequest);
    lastWheelFeedback = motors.updateFeedback(feedbackMode);
    runtimeStats.recordFeedbackRefresh(feedbackMode == MotorFeedbackMode::Full);
  } else {
    lastWheelFeedback = WheelFeedback();
  }

  const RobotMode previousMode = robotState.mode();
  robotState.update(frame, command);
  if (command.stop) {
    command.stop = false;
  }
  handleAutoArm(frame, previousMode);
  const RobotMode currentMode = robotState.mode();
  if (previousMode == RobotMode::Balancing && currentMode == RobotMode::Fault) {
    lastStopReason =
        frame.gyroFresh ? StopReason::FallFault : StopReason::GyroFault;
  } else if (previousMode == RobotMode::Calibrating &&
             currentMode == RobotMode::Fault) {
    lastStopReason = frame.gyroFresh ? StopReason::CalibrationFault
                                     : StopReason::GyroFault;
  }
  if (previousMode != currentMode && currentMode == RobotMode::Balancing) {
    lastStopReason = StopReason::None;
    motors.resetTravel();
    if (Config::EnableMotorFeedback) {
      lastWheelFeedback = motors.updateFeedback(MotorFeedbackMode::Full);
      runtimeStats.recordFeedbackRefresh(true);
    } else {
      lastWheelFeedback = WheelFeedback();
    }
    balance.reset();
    lqr.reset();
    motorOutputLatch.reset();
    safetyCutoff.reset();
    lastBalanceOutput = 0;
    balancingStartMillis = nowMillis;
    if (!balanceSessionUsesPersistedPoint) {
      activeBalancePointDegrees =
          robotState.uprightAngleDegrees() + currentTrimDegrees;
    }
    balancePointLearner.reset(activeBalancePointDegrees, nowMillis);
  }

  if (robotState.motorsEnabled()) {
    BalancePipelineInput pipelineInput;
    pipelineInput.frame = frame;
    pipelineInput.wheelFeedback = lastWheelFeedback;
    pipelineInput.uprightAngleDegrees = robotState.uprightAngleDegrees();
    pipelineInput.activeBalancePointDegrees = activeBalancePointDegrees;
    pipelineInput.currentTrimDegrees = currentTrimDegrees;
    pipelineInput.balancingStartMillis = balancingStartMillis;
    pipelineInput.balanceSessionUsesPersistedPoint =
        balanceSessionUsesPersistedPoint;
    pipelineInput.dtSeconds = dtSeconds;

    const BalancePipelineOutput pipelineOutput =
        balancePipeline.update(pipelineInput, balance, lqr);
    lastTargetAngle = pipelineOutput.targetAngleDegrees;
    lastRawBalanceOutput = pipelineOutput.rawBalanceOutput;
    lastBalanceOutput = pipelineOutput.balanceOutput;
    lastTravelHoldTargetCorrection =
        pipelineOutput.travelHoldTargetCorrectionDegrees;
    const float safetyAngleError =
        pipelineOutput.targetAngleDegrees - frame.angleDegrees;
    if (safetyCutoff.update(safetyAngleError, pipelineOutput.balanceOutput,
                            nowMillis)) {
      applyStopCommand(nowMillis, StopReason::SafetyCutoff);
      safetyCutoff.reset();
    } else {
      updateBalancePointLearning(frame, pipelineOutput.baseTargetDegrees,
                                 pipelineOutput.balanceOutput, nowMillis);
      lastMotorOutput = pipelineOutput.motorCommand;
      if (motorOutputLatch.shouldWrite(lastMotorOutput)) {
        motors.write(lastMotorOutput);
        runtimeStats.recordMotorWrite();
      }
    }
  } else if (motorTestActive(nowMillis)) {
    if (motorOutputLatch.shouldWrite(lastMotorOutput)) {
      motors.write(lastMotorOutput);
      runtimeStats.recordMotorWrite();
    }
  } else {
    safetyCutoff.reset();
    balance.reset();
    lqr.reset();
    if (motorOutputLatch.shouldStop()) {
      motors.stop();
      runtimeStats.recordMotorStop();
    }
    // Robot is idle here, so it is safe to perform any deferred EEPROM commit.
    flushPendingBalancePoint();
    lastTargetAngle = robotState.uprightAngleDegrees();
    lastRawBalanceOutput = 0;
    lastBalanceOutput = 0;
    lastTravelHoldTargetCorrection = 0.0f;
    lastMotorOutput = MotorCommand();
  }

  lastFrame = frame;
  updateStatusLed(nowMillis);
  printModeChangeIfNeeded();
  if (Config::EnableDebugSerial || Config::EnableBluetoothTestControl) {
    const unsigned long telemetryStartMicros = micros();
    printDebug(lastFrame);
    const unsigned long telemetryMicros = micros() - telemetryStartMicros;
    if (telemetryMicros > 0) {
      runtimeStats.recordTelemetryPrint(telemetryMicros);
    }
  }
  runtimeStats.recordBalanceTick(elapsedMicros, micros() - workStartMicros,
                                 Config::BalanceLoopMicros);
}

void handleAutoArm(const SensorFrame& frame, RobotMode modeBeforeAutoArm) {
  if (!runtimeAutoArmEnabled) {
    return;
  }

  if (modeBeforeAutoArm != RobotMode::Disarmed ||
      robotState.mode() != RobotMode::Disarmed ||
      manualArmAttemptIsFresh(frame.nowMillis) || command.stop ||
      motorTestActive(frame.nowMillis)) {
    autoArm.reset();
    return;
  }

  if (!autoArm.update(frame)) {
    return;
  }

  activeBalancePointDegrees =
      Config::EnableBalancePointLearning
          ? balancePointStore.balancePointDegrees()
          : Config::AutoArmDefaultBalancePointDegrees;
  if (robotState.startBalancingAt(activeBalancePointDegrees)) {
    balanceSessionUsesPersistedPoint = true;
    if (Config::EnableDebugSerial) {
      Serial.print(F("auto-arm balancePoint="));
      Serial.println(activeBalancePointDegrees);
    }
    if (Config::EnableBluetoothTestControl && bluetoothTelemetryEnabled) {
      ROBOT_BLUETOOTH_SERIAL.print(F("auto-arm balancePoint="));
      ROBOT_BLUETOOTH_SERIAL.println(activeBalancePointDegrees);
    }
  }
}

bool motorTestActive(unsigned long nowMillis) {
  return static_cast<long>(motorTestUntilMillis - nowMillis) > 0;
}

bool manualCommandsSuppressed(unsigned long nowMillis) {
  return static_cast<long>(manualCommandSuppressedUntilMillis - nowMillis) > 0;
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

  if (result.balancePointDegrees < Config::MinPersistedBalancePointDegrees ||
      result.balancePointDegrees > Config::MaxPersistedBalancePointDegrees) {
    return;
  }

  // Apply the learned point in RAM immediately so balancing and telemetry use
  // it right away, but defer the blocking EEPROM commit until the robot is
  // disarmed (see flushPendingBalancePoint) so it never stalls the balance loop.
  activeBalancePointDegrees = result.balancePointDegrees;
  autoArm.setTargetBalancePoint(activeBalancePointDegrees);
  pendingBalancePointDegrees = result.balancePointDegrees;
  balancePointSavePending = true;
  if (Config::EnableDebugSerial) {
    Serial.print(F("balance-point learned="));
    Serial.println(activeBalancePointDegrees);
  }
  if (Config::EnableBluetoothTestControl && bluetoothTelemetryEnabled) {
    ROBOT_BLUETOOTH_SERIAL.print(F("balance-point learned="));
    ROBOT_BLUETOOTH_SERIAL.println(activeBalancePointDegrees);
  }
}

// Commit a learned balance point to EEPROM.  Only called while the robot is not
// balancing, because EEPROM writes block for several ms per changed byte and
// would otherwise stall the real-time control loop.
void flushPendingBalancePoint() {
  if (!balancePointSavePending) {
    return;
  }
  balancePointSavePending = false;
  balancePointStore.saveBalancePoint(pendingBalancePointDegrees);
}

void readCommands(unsigned long nowMillis) {
  if (Config::EnableBluetoothTestControl) {
    if (readCommandsFrom(bluetoothCommandReader, ROBOT_BLUETOOTH_SERIAL,
                         nowMillis)) {
      return;
    }
  }
  readCommandsFrom(usbCommandReader, Serial, nowMillis);
}

bool readCommandsFrom(CommandReader& reader, Stream& reply,
                      unsigned long nowMillis) {
  ParsedCommand parsed;
  if (reader.readCommand(parsed, nowMillis)) {
    return applyParsedCommand(parsed, reply);
  }
  return false;
}

bool applyParsedCommand(const ParsedCommand& parsed, Stream& reply) {
  switch (parsed.action) {
  case ParsedCommandAction::None:
    return false;
  case ParsedCommandAction::Invalid:
    reply.println(F("error command"));
    return false;
  case ParsedCommandAction::Arm:
    if (command.stop || motorTestActive(parsed.receivedMillis) ||
        manualCommandsSuppressed(parsed.receivedMillis)) {
      reply.println(F("ignored arm"));
      return false;
    }
    command.arm = true;
    command.stop = false;
    command.receivedMillis = parsed.receivedMillis;
    reply.println(F("ok arm"));
    return false;
  case ParsedCommandAction::Stop:
    applyStopCommand(parsed.receivedMillis, StopReason::Command);
    reply.println(F("ok stop"));
    return true;
  case ParsedCommandAction::MotorBackward:
    reply.println(startMotorTest(Config::MotorTestCommand,
                                 parsed.receivedMillis)
                      ? F("ok m+")
                      : F("ignored m+"));
    return false;
  case ParsedCommandAction::MotorForward:
    reply.println(startMotorTest(-Config::MotorTestCommand,
                                 parsed.receivedMillis)
                      ? F("ok m-")
                      : F("ignored m-"));
    return false;
  case ParsedCommandAction::SetTrim:
    applyRuntimeTrim(parsed.first);
    reply.print(F("ok trim="));
    reply.println(currentTrimDegrees);
    return false;
  case ParsedCommandAction::SetPid:
    applyRuntimePid(parsed.first, parsed.second, parsed.third);
    reply.print(F("ok pid kp="));
    reply.print(currentKp);
    reply.print(F(" ki="));
    reply.print(currentKi);
    reply.print(F(" kd="));
    reply.println(currentKd);
    return false;
  case ParsedCommandAction::BalancePointQuery:
    printBalancePointStatusTo(reply);
    return false;
  case ParsedCommandAction::BalancePointSet:
    setPersistedBalancePoint(parsed.first, reply, parsed.receivedMillis);
    return false;
  case ParsedCommandAction::BalancePointClear:
    clearPersistedBalancePoint(reply, parsed.receivedMillis);
    return true;
  case ParsedCommandAction::AutoOn:
    runtimeAutoArmEnabled = Config::EnableAutoArm;
    reply.println(runtimeAutoArmEnabled ? F("ok auto=on")
                                        : F("ok auto=disabled-by-config"));
    return false;
  case ParsedCommandAction::AutoOff:
    runtimeAutoArmEnabled = false;
    autoArm.suppressUntil(parsed.receivedMillis,
                          Config::AutoArmStopCooldownMillis);
    reply.println(F("ok auto=off"));
    return false;
  case ParsedCommandAction::LearnOn:
    if (Config::EnableBalancePointLearning) {
      balancePointLearningEnabled = true;
      balancePointLearner.reset(balancePointStore.balancePointDegrees(),
                                parsed.receivedMillis);
      reply.println(F("ok learn=on"));
    } else {
      reply.println(F("ok learn=disabled-by-config"));
    }
    return false;
  case ParsedCommandAction::LearnOff:
    balancePointLearningEnabled = false;
    reply.println(F("ok learn=off"));
    return false;
  case ParsedCommandAction::Status:
    if (Config::EnableMotorFeedback) {
      lastWheelFeedback = motors.updateFeedback(MotorFeedbackMode::Full);
      runtimeStats.recordFeedbackRefresh(true);
    }
    printStatus(reply, lastFrame);
    return false;
  case ParsedCommandAction::TelemetryOn:
    bluetoothTelemetryEnabled = true;
    reply.println(F("ok telem=on"));
    return false;
  case ParsedCommandAction::TelemetryOff:
    bluetoothTelemetryEnabled = false;
    reply.println(F("ok telem=off"));
    return false;
  }
  return false;
}

void applyStopCommand(unsigned long nowMillis, StopReason reason) {
  command.arm = false;
  command.stop = true;
  command.receivedMillis = nowMillis;
  lastStopReason = reason;
  motorTestUntilMillis = 0;
  manualCommandSuppressedUntilMillis =
      nowMillis + Config::AutoArmStopCooldownMillis;
  if (motorOutputLatch.shouldStop()) {
    motors.stop();
    runtimeStats.recordMotorStop();
  }
  safetyCutoff.reset();
  lastMotorOutput = MotorCommand();
  lastRawBalanceOutput = 0;
  lastBalanceOutput = 0;
  lastTravelHoldTargetCorrection = 0.0f;
  autoArm.suppressUntil(nowMillis, Config::AutoArmStopCooldownMillis);
  balanceSessionUsesPersistedPoint = false;
  resetCommandInputs();
}

void resetCommandInputs() {
  drainCommandStream(Serial);
  usbCommandReader.reset();
  if (Config::EnableBluetoothTestControl) {
    drainCommandStream(ROBOT_BLUETOOTH_SERIAL);
    bluetoothCommandReader.reset();
  }
}

void drainCommandStream(Stream& stream) {
  while (stream.available() > 0) {
    if (stream.read() < 0) {
      return;
    }
  }
}

void clearPersistedBalancePoint(Stream& reply, unsigned long nowMillis) {
  if (!Config::EnableBalancePointLearning) {
    reply.println(F("ok bp-clear=disabled-by-config"));
    return;
  }

  applyStopCommand(nowMillis);
  balancePointStore.clearBalancePoint(Config::AutoArmDefaultBalancePointDegrees);
  balancePointSavePending = false;  // manual clear supersedes any deferred learn
  activeBalancePointDegrees = balancePointStore.balancePointDegrees();
  autoArm.setTargetBalancePoint(activeBalancePointDegrees);
  balancePointLearner.reset(activeBalancePointDegrees, nowMillis);
  reply.print(F("ok bp-clear default="));
  reply.println(activeBalancePointDegrees);
}

void setPersistedBalancePoint(float angleDegrees, Stream& reply,
                              unsigned long nowMillis) {
  if (!Config::EnableBalancePointLearning) {
    reply.println(F("ok bp=disabled-by-config"));
    return;
  }

  if (robotState.mode() != RobotMode::Disarmed || command.stop ||
      motorTestActive(nowMillis) || manualArmAttemptIsFresh(nowMillis) ||
      manualCommandsSuppressed(nowMillis)) {
    reply.println(F("error bp-busy"));
    return;
  }

  if (!balancePointStore.saveBalancePoint(angleDegrees)) {
    reply.println(F("error bp-range"));
    return;
  }
  balancePointSavePending = false;  // manual set supersedes any deferred learn
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

bool startMotorTest(int16_t output, unsigned long nowMillis) {
  if (robotState.mode() != RobotMode::Disarmed ||
      manualCommandsSuppressed(nowMillis)) {
    return false;
  }

  command.arm = false;
  lastMotorOutput.left = output;
  lastMotorOutput.right = output;
  motorTestUntilMillis = nowMillis + Config::MotorTestMillis;
  return true;
}

TelemetrySnapshot buildTelemetrySnapshot(const SensorFrame& frame) {
  TelemetrySnapshot snapshot;
  snapshot.mode = robotState.mode();
  snapshot.loopMicros = lastLoopMicros;
  snapshot.angleDegrees = frame.angleDegrees;
  snapshot.uprightAngleDegrees = robotState.uprightAngleDegrees();
  snapshot.targetAngleDegrees = lastTargetAngle;
  snapshot.errorDegrees = Config::EnableLqrController
                              ? lqr.lastErrorDegrees()
                              : balance.lastErrorDegrees();
  snapshot.trimDegrees = currentTrimDegrees;
  snapshot.activeBalancePointDegrees = activeBalancePointDegrees;
  snapshot.storedBalancePoint = balancePointStore.hasStoredBalancePoint();
  snapshot.stopReason = lastStopReason;
  snapshot.resetRaw = resetCauseRaw;
  snapshot.autoArmEnabled = runtimeAutoArmEnabled;
  snapshot.autoAngleErrorDegrees = autoArm.angleErrorDegrees(frame);
  snapshot.gyroFresh = frame.gyroFresh;
  snapshot.gyroRateDegPerSec = frame.angleRateDegPerSec;
  snapshot.filteredRateDegPerSec =
      Config::EnableLqrController
          ? lqr.lastMeasuredAngleRateDegreesPerSecond()
          : balance.lastMeasuredAngleRateDegreesPerSecond();
  snapshot.rawBalanceOutput = lastRawBalanceOutput;
  snapshot.balanceOutput = lastBalanceOutput;
  snapshot.averageSpeedRpm = lastWheelFeedback.averageSpeedRpm;
  snapshot.averagePositionDegrees = lastWheelFeedback.averagePositionDegrees;
  snapshot.travelHoldTargetCorrectionDegrees =
      lastTravelHoldTargetCorrection;
  snapshot.leftMotor = lastMotorOutput.left;
  snapshot.rightMotor = lastMotorOutput.right;
  snapshot.leftPwm = lastWheelFeedback.leftPwm;
  snapshot.rightPwm = lastWheelFeedback.rightPwm;
  snapshot.kp = currentKp;
  snapshot.ki = currentKi;
  snapshot.kd = currentKd;
  snapshot.runtime = runtimeStats.snapshot();
  return snapshot;
}

void printStatus(Stream& out, const SensorFrame& frame) {
  TelemetryFormatter::printStatus(out, buildTelemetrySnapshot(frame));
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
  if (Config::EnableDebugSerial &&
      frame.nowMillis - lastDebugMillis >= Config::DebugPeriodMillis) {
    lastDebugMillis = frame.nowMillis;
    printDebugTo(Serial, frame);
  }

  if (Config::EnableBluetoothTestControl && bluetoothTelemetryEnabled &&
      frame.nowMillis - lastBluetoothTelemetryMillis >=
          Config::BluetoothTelemetryPeriodMillis) {
    lastBluetoothTelemetryMillis = frame.nowMillis;
    printBluetoothTelemetryTo(ROBOT_BLUETOOTH_SERIAL, frame);
  }
}

void printDebugTo(Stream& out, const SensorFrame& frame) {
  TelemetryFormatter::printDebug(out, buildTelemetrySnapshot(frame));
}

void printBluetoothTelemetryTo(Stream& out, const SensorFrame& frame) {
  TelemetryFormatter::printBluetoothTelemetry(out,
                                              buildTelemetrySnapshot(frame));
}

void setStatusLed(bool on) {
  if (statusLedOn == on) {
    return;
  }

  statusLedOn = on;
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
}

bool updateReasonBlink(unsigned long nowMillis, StopReason reason) {
  unsigned long periodMillis = 0;
  switch (reason) {
  case StopReason::SafetyCutoff:
    periodMillis = 120UL;
    break;
  case StopReason::FallFault:
  case StopReason::GyroFault:
  case StopReason::CalibrationFault:
    periodMillis = 500UL;
    break;
  case StopReason::None:
  case StopReason::Command:
    return false;
  }

  if (nowMillis - lastStatusLedToggleMillis >= periodMillis) {
    lastStatusLedToggleMillis = nowMillis;
    setStatusLed(!statusLedOn);
  }
  return true;
}

void updateStatusLed(unsigned long nowMillis) {
  switch (robotState.mode()) {
  case RobotMode::Disarmed:
    if (updateReasonBlink(nowMillis, lastStopReason)) {
      break;
    }
    setStatusLed(false);
    break;
  case RobotMode::Calibrating:
    if (nowMillis - lastStatusLedToggleMillis >= 100UL) {
      lastStatusLedToggleMillis = nowMillis;
      setStatusLed(!statusLedOn);
    }
    break;
  case RobotMode::Balancing:
    setStatusLed(true);
    break;
  case RobotMode::Fault:
    updateReasonBlink(nowMillis, lastStopReason);
    break;
  }
}
