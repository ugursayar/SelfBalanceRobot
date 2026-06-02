#include "AutoArmController.h"
#include "BalanceController.h"
#include "BalancePipeline.h"
#include "BalancePointLearner.h"
#include "BalancePointStore.h"
#include "BluetoothSerialPort.h"
#include "CommandReader.h"
#include "EepromByteStorage.h"
#include "Motors.h"
#include "ResetDiagnostics.h"
#include "RobotState.h"
#include "RuntimeStats.h"
#include "Sensors.h"
#include "config.h"

#include <MeMegaPi.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <string.h>

uint8_t resetCauseRaw __attribute__((section(".noinit")));
void captureResetCause() __attribute__((naked)) __attribute__((section(".init3")));
void captureResetCause() {
  resetCauseRaw = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

Sensors sensors;
Motors motors;
BalanceController balance;
BalancePipeline balancePipeline;
RobotState robotState;
EepromByteStorage eepromStorage;
BalancePointStore balancePointStore(eepromStorage,
                                    Config::BalancePointEepromAddress);
AutoArmController autoArm;
BalancePointLearner balancePointLearner;
CommandReader usbCommandReader;
CommandReader bluetoothCommandReader;
RuntimeStats runtimeStats;

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
bool readCommandsFrom(CommandReader& reader, Stream& reply,
                      unsigned long nowMillis);
bool applyParsedCommand(const ParsedCommand& parsed, Stream& reply);
void applyStopCommand(unsigned long nowMillis);
void resetCommandInputs();
void drainCommandStream(Stream& stream);
void clearPersistedBalancePoint(Stream& reply, unsigned long nowMillis);
void setPersistedBalancePoint(float angleDegrees, Stream& reply,
                              unsigned long nowMillis);
void configureAutoArmAndLearning();
void printBalancePointStatus(bool loaded);
void printBalancePointStatusTo(Stream& out);
void handleAutoArm(const SensorFrame& frame, RobotMode modeBeforeAutoArm);
bool motorTestActive(unsigned long nowMillis);
bool manualCommandsSuppressed(unsigned long nowMillis);
bool manualArmAttemptIsFresh(unsigned long nowMillis);
void updateBalancePointLearning(const SensorFrame& frame,
                                float baseTargetDegrees,
                                int16_t balanceOutput,
                                unsigned long nowMillis);
void applyRuntimePid(float kp, float ki, float kd);
void applyRuntimeTrim(float trimDegrees);
bool startMotorTest(int16_t output, unsigned long nowMillis);
void printStatus(Stream& out, const SensorFrame& frame);
void printResetCause(Stream& out);
void printResetFlag(Stream& out, bool present, const __FlashStringHelper* label,
                    bool& hasPrinted);
void printMode(Stream& out, RobotMode mode);
void printModeChangeIfNeeded();
void printDebug(const SensorFrame& frame);
void printDebugTo(Stream& out, const SensorFrame& frame);
void printBluetoothTelemetryTo(Stream& out, const SensorFrame& frame);
void updateStatusLed(unsigned long nowMillis);

void setup() {
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
  lastWheelFeedback = motors.updateFeedback();
  runtimeStats.recordFeedbackRefresh(true);

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
    runtimeStats.recordFeedbackRefresh(true);
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
        balancePipeline.update(pipelineInput, balance);
    lastTargetAngle = pipelineOutput.targetAngleDegrees;
    lastRawBalanceOutput = pipelineOutput.rawBalanceOutput;
    lastBalanceOutput = pipelineOutput.balanceOutput;
    lastTravelHoldTargetCorrection =
        pipelineOutput.travelHoldTargetCorrectionDegrees;
    updateBalancePointLearning(frame, pipelineOutput.baseTargetDegrees,
                               pipelineOutput.balanceOutput, nowMillis);
    lastMotorOutput = pipelineOutput.motorCommand;
    motors.write(lastMotorOutput);
    runtimeStats.recordMotorWrite();
  } else if (motorTestActive(nowMillis)) {
    motors.write(lastMotorOutput);
    runtimeStats.recordMotorWrite();
  } else {
    balance.reset();
    motors.stop();
    runtimeStats.recordMotorStop();
    lastTargetAngle = robotState.uprightAngleDegrees();
    lastRawBalanceOutput = 0;
    lastBalanceOutput = 0;
    lastTravelHoldTargetCorrection = 0.0f;
    lastMotorOutput = MotorCommand();
  }

  lastFrame = frame;
  updateStatusLed(nowMillis);
  printModeChangeIfNeeded();
  const unsigned long telemetryStartMicros = micros();
  printDebug(lastFrame);
  const unsigned long telemetryMicros = micros() - telemetryStartMicros;
  if (telemetryMicros > 0) {
    runtimeStats.recordTelemetryPrint(telemetryMicros);
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

  activeBalancePointDegrees = balancePointStore.balancePointDegrees();
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
    ROBOT_BLUETOOTH_SERIAL.print(F("balance-point saved="));
    ROBOT_BLUETOOTH_SERIAL.print(activeBalancePointDegrees);
    ROBOT_BLUETOOTH_SERIAL.print(F(" writes="));
    ROBOT_BLUETOOTH_SERIAL.println(balancePointStore.writeCounter());
  }
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
    applyStopCommand(parsed.receivedMillis);
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
    balancePointLearningEnabled = true;
    balancePointLearner.reset(balancePointStore.balancePointDegrees(),
                              parsed.receivedMillis);
    reply.println(F("ok learn=on"));
    return false;
  case ParsedCommandAction::LearnOff:
    balancePointLearningEnabled = false;
    reply.println(F("ok learn=off"));
    return false;
  case ParsedCommandAction::Status:
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

void applyStopCommand(unsigned long nowMillis) {
  command.arm = false;
  command.stop = true;
  command.receivedMillis = nowMillis;
  motorTestUntilMillis = 0;
  manualCommandSuppressedUntilMillis =
      nowMillis + Config::AutoArmStopCooldownMillis;
  motors.stop();
  runtimeStats.recordMotorStop();
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
  out.print(F(" reset="));
  printResetCause(out);
  out.print(F(" resetRaw=0x"));
  out.print(resetCauseRaw, HEX);
  out.print(F(" auto="));
  out.print(runtimeAutoArmEnabled ? F("on") : F("off"));
  out.print(F(" autoErr="));
  out.print(autoArm.angleErrorDegrees(frame));
  out.print(F(" gyroFresh="));
  out.print(frame.gyroFresh ? F("yes") : F("no"));
  out.print(F(" gyroRate="));
  out.print(frame.angleRateDegPerSec);
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
  out.print(currentKd);
  const RuntimeStatsSnapshot runtime = runtimeStats.snapshot();
  out.print(F(" loopTicks="));
  out.print(runtime.balanceTicks);
  out.print(F(" workUs="));
  out.print(runtime.lastWorkMicros);
  out.print(F(" maxWorkUs="));
  out.print(runtime.maxWorkMicros);
  out.print(F(" missed="));
  out.print(runtime.missedDeadlines);
  out.print(F(" feedbackFull="));
  out.print(runtime.fullFeedbackRefreshes);
  out.print(F(" feedbackLight="));
  out.print(runtime.lightFeedbackRefreshes);
  out.print(F(" motorWrites="));
  out.print(runtime.motorWrites);
  out.print(F(" motorStops="));
  out.print(runtime.motorStops);
  out.print(F(" telemUs="));
  out.print(runtime.lastTelemetryMicros);
  out.print(F(" maxTelemUs="));
  out.println(runtime.maxTelemetryMicros);
}

void printResetCause(Stream& out) {
  const ResetDiagnostics::ResetCauseFlags flags =
      ResetDiagnostics::decode(resetCauseRaw);
  bool hasPrinted = false;
  printResetFlag(out, flags.powerOn, F("por"), hasPrinted);
  printResetFlag(out, flags.external, F("ext"), hasPrinted);
  printResetFlag(out, flags.brownOut, F("bor"), hasPrinted);
  printResetFlag(out, flags.watchdog, F("wdt"), hasPrinted);
  printResetFlag(out, flags.jtag, F("jtag"), hasPrinted);
  printResetFlag(out, flags.unknown, F("unknown"), hasPrinted);
  if (!hasPrinted) {
    out.print(F("none"));
  }
}

void printResetFlag(Stream& out, bool present, const __FlashStringHelper* label,
                    bool& hasPrinted) {
  if (!present) {
    return;
  }
  if (hasPrinted) {
    out.print(F(","));
  }
  out.print(label);
  hasPrinted = true;
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

void printBluetoothTelemetryTo(Stream& out, const SensorFrame& frame) {
  out.print(F("telem mode="));
  printMode(out, robotState.mode());
  out.print(F(" angle="));
  out.print(frame.angleDegrees);
  out.print(F(" target="));
  out.print(lastTargetAngle);
  out.print(F(" auto="));
  out.print(runtimeAutoArmEnabled ? F("on") : F("off"));
  out.print(F(" autoErr="));
  out.print(autoArm.angleErrorDegrees(frame));
  out.print(F(" gyroFresh="));
  out.print(frame.gyroFresh ? F("yes") : F("no"));
  out.print(F(" gyroRate="));
  out.print(frame.angleRateDegPerSec);
  out.print(F(" rate="));
  out.print(balance.lastMeasuredAngleRateDegreesPerSecond());
  out.print(F(" balance="));
  out.print(lastBalanceOutput);
  out.print(F(" left="));
  out.print(lastMotorOutput.left);
  out.print(F(" right="));
  out.println(lastMotorOutput.right);
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
