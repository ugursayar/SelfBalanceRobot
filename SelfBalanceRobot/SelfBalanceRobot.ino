#include "BalanceController.h"
#include "BluetoothControl.h"
#include "DriveMixer.h"
#include "Motors.h"
#include "RobotState.h"
#include "Sensors.h"
#include "config.h"

#include <MeMegaPi.h>

Sensors sensors;
Motors motors;
BluetoothControl bluetooth;
BluetoothControl usbCommands;
BalanceController balance;
DriveMixer mixer;
RobotState robotState;

unsigned long lastBalanceMicros = 0;
unsigned long lastDebugMillis = 0;
SensorFrame lastFrame;
ControlCommand lastCommand;
MotorCommand lastMotorOutput;
float lastTargetAngle = 0.0f;
int16_t lastBalanceOutput = 0;
float currentKp = Config::BalanceKp;
float currentKd = Config::BalanceKd;
RobotMode lastReportedMode = RobotMode::Disarmed;

void printDebug(const SensorFrame& frame, const ControlCommand& command,
                const MotorCommand& motorOutput);
void printModeChangeIfNeeded();
bool commandAIsNewerOrSame(const ControlCommand& a, const ControlCommand& b);
int16_t applyMinimumBalanceCommand(int16_t balanceOutput, float angleError);

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  sensors.begin();
  motors.begin();
  bluetooth.begin(Serial1);
  usbCommands.begin(Serial);

  balance.setTunings(Config::BalanceKp, Config::BalanceKi, Config::BalanceKd);
  balance.setGainSchedule(Config::SmallErrorDegrees,
                          Config::SmallErrorGainScale);
  balance.setOutputLimit(Config::MaxMotorCommand);

  mixer.setLimits(Config::MaxMotorCommand, Config::MaxDriveCommand,
                  Config::MaxTurnCommand, Config::MotorDeadband);

  robotState.configure(Config::FallAngleDegrees,
                       Config::StillAngleDeltaDegrees,
                       Config::ObstacleStopDistanceCm,
                       Config::CalibrationMillis,
                       Config::CommandTimeoutMillis);

  lastBalanceMicros = micros();

  if (Config::EnableDebugSerial) {
    Serial.println(F("SelfBalanceRobot ready. Send commands over Bluetooth or USB Serial Monitor with newline."));
  }
}

void loop() {
  const unsigned long nowMicros = micros();
  const unsigned long elapsedMicros = nowMicros - lastBalanceMicros;
  if (elapsedMicros < Config::BalanceLoopMicros) {
    return;
  }
  lastBalanceMicros = nowMicros;

  const float dtSeconds = static_cast<float>(elapsedMicros) * 0.000001f;
  const unsigned long nowMillis = millis();

  const SensorFrame& frame = sensors.update(nowMillis);
  const ControlCommand& bluetoothCommand = bluetooth.update(nowMillis);
  const ControlCommand& usbCommand = usbCommands.update(nowMillis);
  const bool useUsbCommand = commandAIsNewerOrSame(usbCommand, bluetoothCommand);
  const ControlCommand& command = useUsbCommand ? usbCommand : bluetoothCommand;

  if (command.hasTuning) {
    currentKp = command.tuneKp;
    currentKd = command.tuneKd;
    balance.setTunings(command.tuneKp, command.tuneKi, command.tuneKd);
    if (useUsbCommand) {
      usbCommands.consumeTuning();
    } else {
      bluetooth.consumeTuning();
    }
  }

  robotState.update(frame, command);

  MotorCommand motorOutput;
  if (robotState.motorsEnabled()) {
    const ControlCommand safe = robotState.safeCommand(command, frame);
    const float uprightAngle = robotState.uprightAngleDegrees();
    const float driveRatio =
        static_cast<float>(safe.forward) /
        static_cast<float>(Config::MaxDriveCommand);
    const float targetAngle =
        uprightAngle + Config::BalanceAngleTrimDegrees +
        driveRatio * Config::MaxTargetLeanDegrees;

    lastTargetAngle = targetAngle;
    balance.setTargetAngle(targetAngle);
    int16_t balanceOutput = balance.update(frame.angleDegrees, dtSeconds);
    if (Config::InvertBalanceOutput) {
      balanceOutput = -balanceOutput;
    }
    balanceOutput =
        applyMinimumBalanceCommand(balanceOutput, targetAngle - frame.angleDegrees);
    lastBalanceOutput = balanceOutput;
    motorOutput = mixer.mix(balanceOutput, 0, safe.turn);
    motors.write(motorOutput);
    lastCommand = safe;
  } else {
    balance.reset();
    motors.stop();
    lastBalanceOutput = 0;
    lastCommand = command;
  }

  lastFrame = frame;
  lastMotorOutput = motorOutput;
  printModeChangeIfNeeded();
  printDebug(lastFrame, lastCommand, lastMotorOutput);
}

bool commandAIsNewerOrSame(const ControlCommand& a, const ControlCommand& b) {
  return static_cast<long>(a.receivedMillis - b.receivedMillis) >= 0;
}

int16_t applyMinimumBalanceCommand(int16_t balanceOutput, float angleError) {
  if (angleError < 0.0f) {
    angleError = -angleError;
  }
  if (angleError < Config::MinBalanceBoostAngleDegrees ||
      balanceOutput == 0) {
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
  case RobotMode::Drive:
    Serial.print(F("drive"));
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
  Serial.println(robotState.uprightAngleDegrees());
}

void printDebug(const SensorFrame& frame, const ControlCommand& command,
                const MotorCommand& motorOutput) {
  if (!Config::EnableDebugSerial) {
    return;
  }

  if (frame.nowMillis - lastDebugMillis < Config::DebugPeriodMillis) {
    return;
  }
  lastDebugMillis = frame.nowMillis;

  Serial.print(F("mode="));
  printMode(robotState.mode());
  Serial.print(F(" angle="));
  Serial.print(frame.angleDegrees);
  Serial.print(F(" upright="));
  Serial.print(robotState.uprightAngleDegrees());
  Serial.print(F(" target="));
  Serial.print(lastTargetAngle);
  Serial.print(F(" distance="));
  Serial.print(frame.distanceCm);
  Serial.print(F(" fwd="));
  Serial.print(command.forward);
  Serial.print(F(" turn="));
  Serial.print(command.turn);
  Serial.print(F(" balance="));
  Serial.print(lastBalanceOutput);
  Serial.print(F(" left="));
  Serial.print(motorOutput.left);
  Serial.print(F(" right="));
  Serial.print(motorOutput.right);
  Serial.print(F(" kp="));
  Serial.print(currentKp);
  Serial.print(F(" kd="));
  Serial.println(currentKd);
}
